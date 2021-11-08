use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::flash::{FlashExt, Parts};
use stm32f1xx_hal::gpio::gpioc::PC13;
use stm32f1xx_hal::gpio;
use stm32f1xx_hal::gpio::{Floating, GpioExt};
use stm32f1xx_hal::pac::{self, Peripherals};
use stm32f1xx_hal::pwm::Channel::C1;
use stm32f1xx_hal::rcc::{Clocks, Rcc, RccExt};
use stm32f1xx_hal::time::U32Ext;
use crate::modules::black_pill::channel_definitions::C13;


use crate::modules::black_pill::pin_config::{Input, Mode, Output, Pin};


mod pin_config;
mod channel_definitions;

/*
Un channel es un pin con traits
los traits se deberian implementar de manera dinamica en en momento de creacion de objeto
nononon simplemente hacher cheks antes de usar el channel usando los modos
Una macro para hacer match del eunm y llamar a la funcion con los argumentos genericos correctos.
 */
// struct Channel<T> {
//     pin: Pin,
//     mode: Mode,
//     channel: T
// }

trait Read {
    fn read(&self) -> u32;
}

fn func() {
    let mut card = BlackPill::new();
    card.set_pin_mode(Pin::C13, Mode::Output(Output::Output_push_pull));
    // let a = card;
}

struct Channels {
    // gpioa: stm32f1xx_hal::gpio::gpioa::Parts,
    // gpiob: stm32f1xx_hal::gpio::gpiob::Parts,
    // gpioc: stm32f1xx_hal::gpio::gpioc::Parts,
    C13: Channel
}

struct BlackPill {
    gpio: Channels,
    delay: Delay,
    crhc: stm32f1xx_hal::gpio::gpioc::CRH,
}


// PC13<stm32f1xx_hal::gpio::Output<PushPull>>


enum Channel {
    C13(channel_definitions::C13::Mode),
    None
}

impl Channel {
    fn into_analog(self, crh: &mut stm32f1xx_hal::gpio::gpioc::CRH) -> Self {
        match self {
            Channel::C13(mode) => {
                if let C13::Mode::Input(input) = mode {
                    if let C13::Input::Analog(p) = input {
                        Channel::C13(C13::Mode::Input(C13::Input::Analog(p.into_analog(crh))))
                    } else { Channel::None }
                } else { Channel::None }
            }
            Channel::None => { Channel::None }
        }
    }
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
        // let b: CRH = gpioc.crh
        BlackPill {
            gpio: Channels {
                // gpioa: peripherals.GPIOA.split(&mut rcc.apb2),
                // gpiob: peripherals.GPIOB.split(&mut rcc.apb2),
                // gpioc: peripherals.GPIOC.split(&mut rcc.apb2),
                C13: Channel::C13(C13::Mode::Input(C13::Input::Analog(gpioc.pc13.into_analog(&mut gpioc.crh)))),
            },
            delay,
            crhc: gpioc.crh,
        }

    }
    //
    // fn init_serial_communication(a: C14<Input<PushPull>> | C15<Input<PusPull>>) {
    //
    // }

    // it shouldn't consume self here...
    fn set_pin_mode<'a>(mut self, pin: pin_config::Pin, mode: pin_config::Mode) -> BlackPill {
        match pin {
            Pin::C13 => {
                match mode {
                    Mode::Input(input) => {
                        match input {
                            pin_config::Input::Analog => {
                                // we need unsafe bc stm32_hal doesn't implement a into_analog
                                // method using a mutable reference which is what is needed here
                                // we are achieving the same result with this unsafe code
                                // let a = self.gpio.C13;

                                self.gpio.C13 = self.gpio.C13.into_analog(&mut self.crhc);
                                self
                                // &mut self.gpio.C13
                            }
                            _ => { self }
                        }
                    }
                    Mode::Output(_) => { self }
                }
            }
        }
    }


    // fn set_pin_mode(&mut self, pin: Pin, mode: Mode) -> Option<Channel> {
    //     // match mode {
    //     //     Mode::Input(input) => None,
    //     //     Mode::Output(_) => {}
    //     // }
    //     match pin {
    //         Pin::C13 => {
    //             match mode {
    //                 Mode::Input(input) => {
    //                     None
    //                     // match input {
    //                     //     Input::Analog => {}
    //                     //     Input::Floating_Input => {}
    //                     //     Input::Input_pull_up => {}
    //                     //     Input::Input_pull_down => {}
    //                     // }
    //                 }
    //                 Mode::Output(output) => {
    //                     match output {
    //                         Output::Output_push_pull => { /* PC13<Output<PushPull>> */
    //                             // Channel::<PC13<stm32f1xx_hal::gpio::Output<PushPull>>>::C13(gpioc.pc13.into_push_pull_output(&mut gpioc.crh))
    //                             Some(Channel::C13(Mode3::Output(gpioc.pc13.into_push_pull_output(&mut gpioc.crh))))
    //                         }
    //                         Output::Alternate_output_drain => {None}
    //                         Output::Alternate_output_open_drain => {None}
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }


    #[allow(dead_code)]
    fn delay(&self, t: u32) {

    }
}

/*

    enum Pin {}
    struct Channel {
        pin: Pin,
        mode: Mode
    }
    let card = BlackPill::new();
    let led = card.set_pin_mode(pin,mode) -> Channel;
    led.set_high();
    card.set_pin_state(C13, HIGH);
    card.start_serial_comunication(pin1: Pin, pin2: Pin) -> ((Tx,Rx), (Channel1, Channel2));
    let ((rx, tx), _) = card.start_serial_comunication(pin1: Pin, pin2: Pin);
    card.set_pwm(pin: Pin, freq: u8 (hz), ratio: u8 (%)) -> Channel;


    enum Input {
        Analog,
        Floating_Input,
        Input_pull_up,
        Input_pull_down,
    }

    enum Output {
        Output_push_pull,
        Alternate_output_drain,
        Alternate_output_open_drain,
    }

    enum Modes {
        Input(Inputs),
        Output(Output)
    }
 */