use stm32f1xx_hal::flash::{FlashExt, Parts};
use stm32f1xx_hal::pac::{self, Peripherals};
use stm32f1xx_hal::rcc::{Clocks, Rcc, RccExt};
use stm32f1xx_hal::time::U32Ext;

mod GPIO_config;

struct BlackPill {
    flash: Parts,
    clocks: Clocks,
}

impl BlackPill {

    /// aaa
    pub fn new() -> BlackPill {
        // Get handles to the hardware objects. These functions can only be called
        // once, so that the borrowchecker can ensure you don't reconfigure
        // something by accident.
        // let perif = pac::Peripherals::take().unwrap();

        let perif: Peripherals = pac::Peripherals::take().unwrap();


        // GPIO pins on the STM32F1 must be driven by the APB2 peripheral clock.
        // This must be enabled first. The HAL provides some abstractions for
        // us: First get a handle to the RCC peripheral:
        // rcc <=> reset and clock control
        let mut rcc: Rcc = perif.RCC.constrain();

        // Now we need a delay object. The delay is of course depending on the clock
        // frequency of the microcontroller, so we need to fix the frequency
        // first. The system frequency is set via the FLASH_ACR register, so we
        // need to get a handle to the FLASH peripheral first:
        let mut flash: Parts = perif.FLASH.constrain();
        // Now we can set the controllers frequency to 8 MHz:
        let clocks: Clocks = rcc.cfgr.sysclk(8.mhz()).freeze(&mut flash.acr);

        BlackPill {
            flash,
            clocks
        }



        // BlackPill {
        //     peripherals: perif,
        //     // clocks
        // }
    }
    fn set_pin_mode(){

    }
}