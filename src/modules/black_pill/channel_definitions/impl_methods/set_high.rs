
// creates the method definition for the Channel enum for a pin ( see example of expansion below )
#[macro_export]
macro_rules! set_high {
    ( $pin: ident, $gpio_group: ident ) => {
        pub fn set_high(&mut self) {
            match self {
                Channel::$pin(mode) => {
                    match mode {
                        $pin::Mode::Output(output) => {
                            match output {
                                $pin::Output::Output_push_pull(p) => {
                                    p.set_high().ok();
                                }
                                $pin::Output::open_drain(p) => {
                                    p.set_high().ok();
                                }
                            }
                        }
                        $pin::Mode::Input(_) => {}
                    }
                }
                Channel::None => {}
            }
        }
    }
}
/*
example of expansion for set_high!(C13, gpic); :

fn set_high(&mut self) {
        match self {
            Channel::C13(mode) => {
                match mode {
                    C13::Mode::Output(output) => {
                        match output {
                            C13::Output::Output_push_pull(p) => {
                                p.set_high().ok();
                            }
                            C13::Output::Alternate_output_drain(p) => {
                                p.set_high().ok();
                            }
                            C13::Output::Alternate_output_open_drain(p) => {
                                p.set_high().ok();
                            }
                        }
                    }
                    C13::Mode::Input(_) => {}
                }
            }
            Channel::None => {}
        }
    }
 */