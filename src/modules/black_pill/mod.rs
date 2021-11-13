use cortex_m_semihosting::hprintln;
use embedded_hal::{
    digital::v2::OutputPin,
    prelude::_embedded_hal_blocking_delay_DelayMs
};
use stm32f1xx_hal::{
    delay::Delay,
    flash::{FlashExt, Parts},
    gpio::{
        self,
        {Floating, GpioExt},
        gpioc::PC13
    },
    pac::{self, Peripherals},
    rcc::{Clocks, Rcc, RccExt},
    time::U32Ext
};

use crate::modules::black_pill::{
    channel_definitions::C13,
    pin_config::{Input, Mode, Output, Pin},
};
use replace_with;
use stm32f1xx_hal::gpio::gpioc::CRH;
use channels::Channel;
use GPIOs::Gpios;

mod pin_config;
mod channel_definitions;
mod GPIOs;
mod channels;

pub fn func() -> ! {
    let mut card = BlackPill::new();
    card.set_pin_mode(Pin::C13, Mode::Output(Output::Output_push_pull));
    loop {
        card.gpio.C13.set_high();
        card.delay.delay_ms(1000_u16);
        card.gpio.C13.set_low();
        card.delay.delay_ms(1000_u16);
    }
}


struct BlackPill {
    gpio: Gpios,
    delay: Delay,
    crhc: stm32f1xx_hal::gpio::gpioc::CRH,
}

impl BlackPill {

    ///
    pub fn new() -> BlackPill {
        // Get handles to the hardware objects. These functions can only be called once
        let peripherals: Peripherals = pac::Peripherals::take().unwrap();
        let cp = cortex_m::Peripherals::take().unwrap();

        // GPIO pins on the STM32F1 must be driven by the APB2 peripheral clock.
        // This must be enabled first. The HAL provides some abstractions for
        // rcc <=> reset and clock control
        let mut rcc: Rcc = peripherals.RCC.constrain();

        // Now we need a delay object. The delay is of course depending on the clock
        // frequency of the microcontroller, so we need to fix the frequency
        // first. The system frequency is set via the FLASH_ACR register, so we
        // need to get a handle to the FLASH peripheral first:
        let mut flash = peripherals.FLASH.constrain();

        // Now we can set the controllers frequency to 8 MHz:
        let clocks = rcc.cfgr.sysclk(8.mhz()).freeze(&mut flash.acr);
        let delay = Delay::new(cp.SYST, clocks);
        let mut gpioc = peripherals.GPIOC.split(&mut rcc.apb2);

        BlackPill {
            gpio: Gpios {
                // C13: Channel::C13(C13::Mode::Input(C13::Input::Analog(gpioc.pc13.into_analog(&mut gpioc.crh)))),
                // B9: Channel::B9(C13::Mode::Input(C13::Input::Analog(gpioc.pc13.into_analog(&mut gpioc.crh)))),
                // B8: Channel,
                // B7: Channel,
                // B6: Channel,
                // B5: Channel,
                // B4: Channel,
                // B3: Channel,
                // A15: Channel,
                // A12: Channel,
                // A11: Channel,
                // A10: Channel,
                // A9: Channel,
                // A8: Channel,
                // B15: Channel,
                // B14: Channel,
                // B13: Channel,
                // B12: Channel,
                C13: Channel::C13(C13::Mode::Input(C13::Input::Analog(gpioc.pc13.into_analog(&mut gpioc.crh)))),
                // C14: Channel,
                // C15: Channel,
                // A0: Channel,
                // A1: Channel,
                // A2: Channel,
                // A3: Channel,
                // A4: Channel,
                // A5: Channel,
                // A6: Channel,
                // A7: Channel,
                // B0: Channel,
                // B1: Channel,
                // B10: Channel,
                // B11: Channel,
                // RESET: Channel,
            },
            delay,
            crhc: gpioc.crh,
        }

    }
    // todo fn init_serial_communication(a: C14<Input<PushPull>> | C15<Input<PusPull>>) { }


    fn set_pin_mode(&mut self, pin: pin_config::Pin, mode: pin_config::Mode) {
        /// Defines a macro to create the necessary code to configure each pin
        /// It just matches the mode for each port and it configures it accordingly
        /// We need to implement the macro inside the method so that it recognises the self keyword,
        /// see the following StackOverflow issue for more info:
        /// https://stackoverflow.com/questions/44120455/how-to-call-methods-on-self-in-macros
        macro_rules! gpio_config {
            ( $pin: ident) => {
                match mode {
                    Mode::Input(input) => {
                        match input {
                            /*
                                we need unsafe bc stm32_hal doesn't implement a into_analog
                                method using a mutable reference which is what is needed here
                                we are achieving the same result with this unsafe code
                                we are using a crate to help with that
                             */
                            pin_config::Input::Analog =>  unsafe {
                                let gpio = &mut self.gpio.C13;
                                let crh: &mut CRH = &mut self.crhc;
                                replace_with::replace_with_or_abort_unchecked(
                                    gpio,
                                    |val| val.into_analog(crh)
                                );
                            },

                            Input::Floating_Input =>  unsafe {
                                let gpio = &mut self.gpio.C13;
                                let crh: &mut CRH = &mut self.crhc;
                                replace_with::replace_with_or_abort_unchecked(
                                    gpio,
                                    |val| val.into_floating_input(crh)
                                );
                            }
                            Input::Input_pull_up => {
                                todo!();
                            }
                            Input::Input_pull_down => {
                                todo!();
                            }
                        }
                    }
                    Mode::Output(output) => {
                        hprintln!("matched output");
                        match output {
                            Output::Output_push_pull => unsafe {
                                let gpio = &mut self.gpio.C13;
                                let crh = &mut self.crhc;
                                replace_with::replace_with_or_abort_unchecked(
                                    gpio,
                                    |val| val.into_Output_push_pull(crh)
                                );
                            },
                            Output::Alternate_output_drain => {
                                todo!();
                            }
                            Output::Alternate_output_open_drain => {
                                todo!();
                            }
                        }
                    }
                }
            };
        }
        match pin {
            Pin::C13 => gpio_config!(C13),
        }
    }

    #[allow(dead_code)]
    fn delay(&mut self, t: u16) {
        self.delay.delay_ms(t);
    }
}
