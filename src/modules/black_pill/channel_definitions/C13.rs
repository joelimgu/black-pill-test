use stm32f1xx_hal::gpio::{self, Analog, Floating, gpioc::PC13, OpenDrain, PullDown, PullUp, PushPull};

pub enum Input {
    Analog(PC13<Analog>),
    Floating_Input(PC13<gpio::Input<Floating>>),
    Input_pull_up(PC13<gpio::Input<PullUp>>),
    Input_pull_down(PC13<gpio::Input<PullDown>>),
}

pub enum Output {
    Output_push_pull(PC13<gpio::Output<PushPull>>),
    // Alternate_output_drain(PC13<gpio::Output<OpenDrain>>),
    Alternate_output_open_drain(PC13<gpio::Output<OpenDrain>>),
}

pub enum Mode {
    Input(Input),
    Output(Output)
}