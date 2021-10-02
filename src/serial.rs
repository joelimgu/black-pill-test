// src/main.rs

// std and main are not available for bare metal software
#![no_std]
#![no_main]

use cortex_m_rt::entry; // The runtime
use embedded_hal::digital::v2::OutputPin; // the `set_high/low`function
use stm32f1xx_hal::{delay::Delay, pac, prelude::*}; // STM32F1 specific functions
#[allow(unused_imports)]
use panic_halt;
use stm32f1xx_hal::rcc::Rcc;
use stm32f1xx_hal::pac::Peripherals;
use stm32f1xx_hal::serial::{Config, Serial}; // When a panic occurs, stop the microcontroller

use nb::block;
use cortex_m_semihosting::hprintln;


// This marks the entrypoint of our application. The cortex_m_rt creates some
// startup code before this, but we don't need to worry about this
#[entry]
fn main() -> ! {
    // Get handles to the hardware objects. These functions can only be called
    // once, so that the borrowchecker can ensure you don't reconfigure
    // something by accident.
    let dp: Peripherals = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // GPIO pins on the STM32F1 must be driven by the APB2 peripheral clock.
    // This must be enabled first. The HAL provides some abstractions for
    // us: First get a handle to the RCC peripheral:
    let mut rcc: Rcc = dp.RCC.constrain();
    // Now we have access to the RCC's registers. The GPIOC can be enabled in
    // RCC_APB2ENR (Prog. Ref. Manual 8.3.7), therefore we must pass this
    // register to the `split` function.
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    // This gives us an exclusive handle to the GPIOC peripheral. To get the
    // handle to a single pin, we need to configure the pin first. Pin C13
    // is usually connected to the Bluepills onboard LED.

    let mut flash = dp.FLASH.constrain();
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

    let sys_clock = rcc.cfgr.sysclk(8.mhz()).freeze(&mut flash.acr);
    // let clocks_serial = rcc.cfgr.freeze(&mut flash.acr);



    // USART1 on Pins A9 and A10
    let pin_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let pin_rx = gpioa.pa10;



    let serial = Serial::usart1(
        dp.USART1,
        (pin_tx, pin_rx),
        &mut afio.mapr,
        Config::default().baudrate(10.bps()),
        sys_clock.clone(),
        &mut rcc.apb2,
    );

    // separate into tx and rx channels
    let (mut tx, _rx) = serial.split();

    let mut delay = Delay::new(cp.SYST, sys_clock);
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    loop {
        led.set_low().ok();
        block!(tx.write(b'A')).ok();
        delay.delay_ms(1_00_u16);
        hprintln!("{:b}", b'A').unwrap();
    }
}
