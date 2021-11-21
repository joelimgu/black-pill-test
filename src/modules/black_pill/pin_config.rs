
/// Enums for configuration purposes only
/// the types defined here are used to define an objective in the set_pin_mode_method

pub enum Pin {
    c13
}

pub enum Input {
    Analog,
    Floating_Input,
    Input_pull_up
}

pub enum Output {
    output_push_pull,
    open_drain
}

pub enum Mode {
    Input(Input),
    Output(Output)
}