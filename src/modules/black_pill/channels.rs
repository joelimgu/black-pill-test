use crate::modules::black_pill::channel_definitions;
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
use crate::{
    into_analog,
    into_output_push_pull,
    set_high,
    set_low,
    into_floating_input,
};


#[derive(Debug)]
pub enum Channel {
    C13(channel_definitions::C13::Mode),
    None
}


impl Channel {

    into_analog!(C13, gpioc);
    into_floating_input!(C13, gpioc);
    // todo pull_up_input
    // todo Input_pull_down // pas dans hal?!

    into_output_push_pull!(C13, gpioc);
    // todo Alternate_output_drain  // pas dans hal?!
    // todo Alternate_output_open_drain

    set_high!(C13, gpic);
    set_low!(C13, gpioc);

    // todo is_high() -> bool
    // todo is_low() -> bool

}
