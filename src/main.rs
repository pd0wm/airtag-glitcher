#![no_std]
#![no_main]
#![feature(asm)]

use cortex_m::asm::delay;
use stm32f1xx_hal::{prelude::*};
use stm32f1xx_hal::gpio::{gpioc::PC13, gpiob::PB7, gpiob::PB8, gpiob::PB9, Output, PushPull, Input, Floating};
use stm32f1xx_hal::time::Hertz;
use stm32f1xx_hal::gpio::{OutputSpeed, IOPinSpeed};

use embedded_hal::digital::v2::{OutputPin, InputPin};

// USB
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use usb_device::bus;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use rtic::cyccnt::U32Ext;
use rtic::app;

#[allow(unused_imports)]
use panic_halt; // When a panic occurs, stop the microcontroller

#[app(device = stm32f1xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        clock_freq: Hertz,
        led: PC13<Output<PushPull>>,
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: SerialPort<'static, UsbBusType>,

        #[init(&mut [0; 9])]
        serial_buf: &'static mut [u8],
        #[init(0)]
        serial_buf_ptr: usize,

        #[init(0)]
        delay : u64,
        #[init(0)]
        width : u64,

        glitch: PB7<Output<PushPull>>,
        meas: PB8<Input<Floating>>,
        pwr: PB9<Output<PushPull>>,
    }

    #[init(spawn = [blink])]
    fn init(cx: init::Context) -> init::LateResources{
        static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;

        let mut core = cx.core;
        core.DWT.enable_cycle_counter();

        let clock_freq : Hertz = 48.mhz().into();

        // Setup clocks
        let mut rcc = cx.device.RCC.constrain();
        let mut flash = cx.device.FLASH.constrain();
        let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(clock_freq).pclk1(24.mhz()).freeze(&mut flash.acr);

        // USB
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.apb2);
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);

        // Reset usb on reflash
        usb_dp.set_low().unwrap();
        delay(clocks.sysclk().0 / 100);

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb = Peripheral {
            usb: cx.device.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        *USB_BUS = Some(UsbBus::new(usb));

        let serial = SerialPort::new(USB_BUS.as_ref().unwrap());

        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0xbbaa, 0xddcc))
            .manufacturer("1337 glitchers")
            .product("Airtag Glitcher")
            .serial_number("12345")
            .device_class(USB_CLASS_CDC)
            .build();

        // LED pins
        let mut gpioc = cx.device.GPIOC.split(&mut rcc.apb2);
        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        cx.spawn.blink().unwrap();

        // Glitch pins
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.apb2);
        let mut glitch = gpiob.pb7.into_push_pull_output(&mut gpiob.crl);
        glitch.set_speed(&mut gpiob.crl, IOPinSpeed::Mhz50);


        let meas = gpiob.pb8.into_floating_input(&mut gpiob.crh);
        let mut pwr = gpiob.pb9.into_push_pull_output(&mut gpiob.crh);
        pwr.set_speed(&mut gpiob.crh, IOPinSpeed::Mhz50);

        pwr.set_low().ok();
        glitch.set_low().ok();

        init::LateResources {
            clock_freq,
            led,
            usb_dev,
            serial,
            glitch,
            meas,
            pwr,
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            core::sync::atomic::spin_loop_hint();
        }
    }

    #[task(resources = [led, clock_freq], schedule = [blink])]
    fn blink(cx: blink::Context) {
        cx.resources.led.toggle().ok();
        let delay : u32 = cx.resources.clock_freq.0 / 5;
        cx.schedule.blink(cx.scheduled + delay.cycles()).unwrap();
    }

    #[task(binds = USB_HP_CAN_TX, resources = [usb_dev, serial, serial_buf_ptr, serial_buf, delay, width, glitch, meas, pwr])]
    fn usb_tx(cx: usb_tx::Context) {
        let mut r = cx.resources;
        usb_poll(&mut r.usb_dev, &mut r.serial,
                 &mut r.serial_buf_ptr, &mut r.serial_buf,
                 &mut r.delay, &mut r.width,
                 &mut r.glitch, &mut r.meas, &mut r.pwr);
    }

    #[task(binds = USB_LP_CAN_RX0, resources = [usb_dev, serial, serial_buf_ptr, serial_buf, delay, width, glitch, meas, pwr])]
    fn usb_rx0(cx: usb_rx0::Context) {
        let mut r = cx.resources;
        usb_poll(&mut r.usb_dev, &mut r.serial,
                 &mut r.serial_buf_ptr, &mut r.serial_buf,
                 &mut r.delay, &mut r.width,
                 &mut r.glitch, &mut r.meas, &mut r.pwr);
    }

    extern "C" {
        fn EXTI0();
    }
};

fn str_to_u64(inp : &[u8]) -> u64 {
    let mut mult : u64 = 1;
    let mut r : u64 = 0;

    for i in (0..inp.len()).rev() {
        r += mult * (inp[i] - '0' as u8) as u64;
        mult *= 10;
    }

    return r;
}

fn usb_poll<B: bus::UsbBus>(
    usb_dev: &mut UsbDevice<'static, B>,
    serial: &mut SerialPort<'static, B>,
    serial_buf_ptr: &mut usize,
    serial_buf: &mut [u8],
    delay : &mut u64,
    width : &mut u64,
    glitch: &mut PB7<Output<PushPull>>,
    meas: &mut PB8<Input<Floating>>,
    pwr: &mut PB9<Output<PushPull>>,

) {
    if !usb_dev.poll(&mut [serial]) {
        return;
    }

    match serial.read(&mut serial_buf[*serial_buf_ptr ..]) {
        Ok(count) => {
            if count == 0 {
                *serial_buf_ptr = 0;
            } else {
                *serial_buf_ptr += count;
                if *serial_buf_ptr == 9 {

                    match serial_buf[0] as char {
                        // Set glitch delay
                        'd' => {
                            *delay = str_to_u64(&serial_buf[1..9]);
                        },
                        // Set glitch width
                        'w' => {
                            *width = str_to_u64(&serial_buf[1..9]);
                        },
                        // Enable power
                        'h' => {
                            pwr.set_high().ok();
                        },
                        // Disable power
                        'l' => {
                            pwr.set_low().ok();
                        }
                        // Glitch!
                        'g' => {
                            // Turn on power
                            pwr.set_high().ok();
                            
                            // Wait for CPU power to be enabled
                            while meas.is_low().unwrap() {
                            }

                            // Delay
                            pwr.set_high().ok();
                            let d = *delay;
                            for _ in 0..d {
                                unsafe {
                                    asm!("nop");
                                }
                            }

                            // Glitch for width
                            let w = *width;

                            unsafe {
                                if w == 0 {
                                    glitch.set_high().ok();
                                    glitch.set_low().ok();
                                } else if w == 1 {
                                    glitch.set_high().ok();
                                    for _ in 0..1 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 2 {
                                    glitch.set_high().ok();
                                    for _ in 0..2 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 3 {
                                    glitch.set_high().ok();
                                    for _ in 0..3 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 4 {
                                    glitch.set_high().ok();
                                    for _ in 0..4 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 5 {
                                    glitch.set_high().ok();
                                    for _ in 0..5 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 6 {
                                    glitch.set_high().ok();
                                    for _ in 0..6 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 7 {
                                    glitch.set_high().ok();
                                    for _ in 0..7 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 8 {
                                    glitch.set_high().ok();
                                    for _ in 0..8 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 9 {
                                    glitch.set_high().ok();
                                    for _ in 0..9 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 10 {
                                    glitch.set_high().ok();
                                    for _ in 0..10 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 11 {
                                    glitch.set_high().ok();
                                    for _ in 0..11 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 12 {
                                    glitch.set_high().ok();
                                    for _ in 0..12 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 13 {
                                    glitch.set_high().ok();
                                    for _ in 0..13 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 14 {
                                    glitch.set_high().ok();
                                    for _ in 0..14 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 15 {
                                    glitch.set_high().ok();
                                    for _ in 0..15 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 16 {
                                    glitch.set_high().ok();
                                    for _ in 0..16 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 17 {
                                    glitch.set_high().ok();
                                    for _ in 0..17 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 18 {
                                    glitch.set_high().ok();
                                    for _ in 0..18 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 19 {
                                    glitch.set_high().ok();
                                    for _ in 0..19 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 20 {
                                    glitch.set_high().ok();
                                    for _ in 0..20 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 21 {
                                    glitch.set_high().ok();
                                    for _ in 0..21 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 22 {
                                    glitch.set_high().ok();
                                    for _ in 0..22 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 23 {
                                    glitch.set_high().ok();
                                    for _ in 0..23 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 24 {
                                    glitch.set_high().ok();
                                    for _ in 0..24 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 25 {
                                    glitch.set_high().ok();
                                    for _ in 0..25 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 26 {
                                    glitch.set_high().ok();
                                    for _ in 0..26 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 27 {
                                    glitch.set_high().ok();
                                    for _ in 0..27 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 28 {
                                    glitch.set_high().ok();
                                    for _ in 0..28 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 29 {
                                    glitch.set_high().ok();
                                    for _ in 0..29 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else if w == 30 {
                                    glitch.set_high().ok();
                                    for _ in 0..30 {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                } else {
                                    glitch.set_high().ok();
                                    for _ in 0..w {
                                        asm!("nop");
                                    }
                                    glitch.set_low().ok();
                                }
                            }


                        },
                        _ => {serial.write(b"unkown command\r\n").ok();},
                    }
                    serial.write(b"OK\r\n").ok();
                    *serial_buf_ptr = 0;
                }
            }

        },
        _ => {}
    }
}
