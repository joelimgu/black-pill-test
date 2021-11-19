
/// Enums for configuration purposes only
/// the types defined here are used to define an objective in the set_pin_mode_method

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