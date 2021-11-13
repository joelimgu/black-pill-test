
// Generates the into_analog method for the Channel enum for a single Pin
//it deconstructs the enum and calls the into_analog method of the hal library
#[macro_export]
macro_rules! into_floating_input {
    ( $pin: ident, $gpio_group: ident ) => {
        pub fn into_floating_input(self, crh: &mut stm32f1xx_hal::gpio::$gpio_group::CRH) -> Self {
            match self {
                Channel::$pin(mode) => {
                    match mode {
                        $pin::Mode::Input(input) => {
                            match input {
                                $pin::Input::Analog(p) => {
                                    Channel::$pin($pin::Mode::Input($pin::Input::Floating_Input(p.into_floating_input(crh))))
                                }
                                $pin::Input::Floating_Input(p) => {
                                    Channel::$pin($pin::Mode::Input($pin::Input::Floating_Input(p.into_floating_input(crh))))
                                }
                                $pin::Input::Input_pull_up(p) => {
                                    Channel::$pin($pin::Mode::Input($pin::Input::Floating_Input(p.into_floating_input(crh))))
                                }
                                $pin::Input::Input_pull_down(p) => {
                                    Channel::$pin($pin::Mode::Input($pin::Input::Floating_Input(p.into_floating_input(crh))))
                                }
                            }
                        }
                        $pin::Mode::Output(output) => {
                            match output {
                                $pin::Output::Output_push_pull(p) => {
                                    Channel::$pin($pin::Mode::Input($pin::Input::Floating_Input(p.into_floating_input(crh))))
                                }

                                $pin::Output::Alternate_output_drain(p) => {
                                    Channel::$pin($pin::Mode::Input($pin::Input::Floating_Input(p.into_floating_input(crh))))
                                }
                                $pin::Output::Alternate_output_open_drain(p) => {
                                    Channel::$pin($pin::Mode::Input($pin::Input::Floating_Input(p.into_floating_input(crh))))
                                }
                            }
                        }
                    }
                }
                Channel::None => { Channel::None }
            }
        }
    }
}

/* example for into_floating_input!(C13, gpioc); :

pub fn into_floating_input(self, crh: &mut stm32f1xx_hal::gpio::gpioc::CRH) -> Self {
        match self {
            Channel::C13(mode) => {
                match mode {
                    C13::Mode::Input(input) => {
                        match input {
                            C13::Input::Analog(p) => {
                                Channel::C13(C13::Mode::Input(C13::Input::Floating_Input(p.into_floating_input(crh))))
                            }
                            C13::Input::Floating_Input(p) => {
                                Channel::C13(C13::Mode::Input(C13::Input::Floating_Input(p.into_floating_input(crh))))
                            }
                            C13::Input::Input_pull_up(p) => {
                                Channel::C13(C13::Mode::Input(C13::Input::Floating_Input(p.into_floating_input(crh))))
                            }
                            C13::Input::Input_pull_down(p) => {
                                Channel::C13(C13::Mode::Input(C13::Input::Floating_Input(p.into_floating_input(crh))))
                            }
                        }
                    }
                    C13::Mode::Output(output) => {
                        match output {
                            C13::Output::Output_push_pull(p) => {
                                Channel::C13(C13::Mode::Input(C13::Input::Floating_Input(p.into_floating_input(crh))))
                            }

                            C13::Output::Alternate_output_drain(p) => {
                                Channel::C13(C13::Mode::Input(C13::Input::Floating_Input(p.into_floating_input(crh))))
                            }
                            C13::Output::Alternate_output_open_drain(p) => {
                                Channel::C13(C13::Mode::Input(C13::Input::Floating_Input(p.into_floating_input(crh))))
                            }
                        }
                    }
                }
            }
            Channel::None => { Channel::None }
        }
    }
 */