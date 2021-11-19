
use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use crate::modules::black_pill::card_struct::BlackPill;
use crate::modules::black_pill::pin_config::{Mode, Output, Pin};

mod pin_config;
mod channel_definitions;
mod GPIOs;
mod channels;
mod card_struct;


pub fn func() -> ! {
    let mut card = BlackPill::new();
    card.set_pin_mode(Pin::C13, Mode::Output(Output::Output_push_pull));
    loop {
        card.gpio.C13.set_high();
        card.delay.delay_ms(1000_u16);
        card.gpio.C13.set_low();
        card.delay.delay_ms(1000_u16);
    }
}
