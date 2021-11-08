
pub enum Pin {
    C13
}

pub enum Input {
    Analog,
    Floating_Input,
    Input_pull_up,
    Input_pull_down,
}

pub enum Output {
    Output_push_pull,
    Alternate_output_drain,
    Alternate_output_open_drain,
}

pub enum Mode {
    Input(Input),
    Output(Output)
}