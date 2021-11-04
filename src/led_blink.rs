// src/main.rs

// std and main are not available for bare metal software
#![no_std]
#![no_main]


use async_cortex_m::task;
use cortex_m_rt::entry; // The runtime
use embedded_hal::digital::v2::OutputPin; // the `set_high/low`function
use stm32f1xx_hal::{delay::Delay, pac, prelude::*}; // STM32F1 specific functions
#[allow(unused_imports)]
use panic_halt;
use stm32f1xx_hal::rcc::Rcc;
use stm32f1xx_hal::pac::Peripherals; // When a panic occurs, stop the microcontroller

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
    // rcc <=> reset and clock control
    //
    let mut rcc: Rcc = dp.RCC.constrain();
    // Now we have access to the RCC's registers. The GPIOC can be enabled in
    // RCC_APB2ENR (Prog. Ref. Manual 8.3.7), therefore we must pass this
    // register to the `split` function.
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    // This gives us an exclusive handle to the GPIOC peripheral. To get the
    // handle to a single pin, we need to configure the pin first. Pin C13
    // is usually connected to the Bluepills onboard LED.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // Now we need a delay object. The delay is of course depending on the clock
    // frequency of the microcontroller, so we need to fix the frequency
    // first. The system frequency is set via the FLASH_ACR register, so we
    // need to get a handle to the FLASH peripheral first:
    let mut flash = dp.FLASH.constrain();
    // Now we can set the controllers frequency to 8 MHz:
    let clocks = rcc.cfgr.sysclk(8.mhz()).freeze(&mut flash.acr);
    // The `clocks` handle ensures that the clocks are now configured and gives
    // the `Delay::new` function access to the configured frequency. With
    // this information it can later calculate how many cycles it has to
    // wait. The function also consumes the System Timer peripheral, so that no
    // other function can access it. Otherwise the timer could be reset during a
    // delay.
    let mut delay = Delay::new(cp.SYST, clocks);

    // Now, enjoy the lightshow!
    loop {
        led.set_high().ok();
        delay.delay_ms(1_000_u16);
        led.set_low().ok();
        delay.delay_ms(1_000_u16);
    }
}
