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