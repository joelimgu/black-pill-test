

// Generates all the necessary enums and traits for a pin ( for an expansion example see below )
#[macro_export]
macro_rules! GPIO_Module {
    ( $pin: ident, $gpio_group: ident ) => {
        use core::fmt::{Debug, Formatter};
        use stm32f1xx_hal::gpio::{
            self, Analog, Floating, $gpio_group::$pin, OpenDrain, PullDown, PullUp, PushPull
        };

        /// Specifies the type of Input the pin is set on and has the underlying hal type associated
        pub enum Input {
            Analog($pin<Analog>),
            Floating_Input($pin<gpio::Input<Floating>>),
            Input_pull_up($pin<gpio::Input<PullUp>>),
        }

        // So that we can hprint the value of the enum for debugging
        impl Debug for Input {
            fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
                match self {
                    Input::Analog(_) => f.write_str("Analog"),
                    Input::Floating_Input(_) => f.write_str("Floating_Input"),
                    Input::Input_pull_up(_) => f.write_str("Input_pull_up"),
                }
            }
        }

        /// Specifies the type of output the pin is set on and has the underlying hal
        /// type associated to it
        pub enum Output {
            Output_push_pull($pin<gpio::Output<PushPull>>),
            open_drain($pin<gpio::Output<OpenDrain>>)
        }

        // So that we can hprint the value of the enum for debugging
        impl Debug for Output {
            fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
                match self {
                    Output::Output_push_pull(_) => f.write_str("Output_push_pull"),
                    Output::open_drain(_) => f.write_str("open_drain"),
                }
            }
        }

        /// Stores if the Pin is configured as an input or output with
        /// a reference to the Input/Output type as an enum
        #[derive(Debug)]
        pub enum Mode {
            Input(Input),
            Output(Output)
        }
    }
}

// Expansion example for the Pin C13 which is in the giopc group:
// It should be used inside a module I didn't include it in the macro bc I thought it
// just made things even more confusing
/*
pub mod C13 {
    use crate::GPIO_Module;
    GPIO_Module!(PC13, gpioc);
}

And this line ( GPIO_Module!(PC13, gpioc); ) expands to:

use core::fmt::{Debug, Formatter, Write};
use stm32f1xx_hal::gpio::{self, Analog, Floating, gpioc::PC13, OpenDrain, PullDown, PullUp, PushPull};


pub enum Input {
    Analog(PC13<Analog>),
    Floating_Input(PC13<gpio::Input<Floating>>),
    Input_pull_up(PC13<gpio::Input<PullUp>>),
    Input_pull_down(PC13<gpio::Input<PullDown>>),
}

impl Debug for Input {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Input::Analog(_) => f.write_str("Analog"),
            Input::Floating_Input(_) => f.write_str("Floating_Input"),
            Input::Input_pull_up(_) => f.write_str("Input_pull_up"),
            Input::Input_pull_down(_) => f.write_str("Input_pull_down")
        }
    }
}

pub enum Output {
    Output_push_pull(PC13<gpio::Output<PushPull>>),
    Alternate_output_drain(PC13<gpio::Output<OpenDrain>>),
    Alternate_output_open_drain(PC13<gpio::Output<OpenDrain>>),
}

impl Debug for Output {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Output::Output_push_pull(_) => f.write_str("Output_push_pull"),
            Output::Alternate_output_drain(_) => f.write_str("Alternate_output_drain"),
            Output::Alternate_output_open_drain(_) => {
                f.write_str("Alternate_output_open_drain")
            }
        }
    }
}

#[derive(Debug)]
pub enum Mode {
    Input(Input),
    Output(Output)
}
 */