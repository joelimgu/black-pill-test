//! Simple CAN example.
//! Requires a transceiver connected to PA11, PA12 (CAN1) or PB5 PB6 (CAN2).

#![no_main]
#![no_std]


use panic_halt as _;

use bxcan::filter::Mask32; //OG
use bxcan::{Frame, StandardId}; //OG
use cortex_m_rt::entry; //OG
use cortex_m_semihosting::hprintln; //OG
use nb::block; //OG
use stm32f1xx_hal::{can::Can, pac, prelude::*}; //OG
use stm32f1xx_hal::timer::Timer; //OG

//For interrupt
//use cortex_m::interrupt::Mutex; //for mutex
//use cortex_m::interrupt as inter; //for interrupt::free
//use cortex_m_rt::interrupt;
use core::cell::RefCell;
use bxcan::Interrupt::Fifo0MessagePending;
use stm32f1xx_hal::pac::interrupt;
use stm32f1::stm32f103;
use core::borrow::Borrow;

// Enable interrupt
use stm32f1::stm32f103::{Interrupt, NVIC};

//Mutex for CAN access
//static CAN_M : Mutex<RefCell<Option<stm32f103::CAN1>>> = Mutex::new(RefCell::new(None));

#[interrupt]
fn USB_LP_CAN_RX0(){
    hprintln!("Hola");
}

#[entry]
//Symbol ! means the fonction returns NEVER => an infinite loop must exist
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    // To meet CAN clock accuracy requirements an external crystal or ceramic
    // resonator must be used. The blue pill has a 8MHz external crystal.
    // Other boards might have a crystal with another frequency or none at all.
    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain();

    let mut can1 = {
        #[cfg(not(feature = "connectivity"))]
            let can = Can::new(dp.CAN1, dp.USB);
        #[cfg(feature = "connectivity")]
            let can = Can::new(dp.CAN1);

        let mut gpioa = dp.GPIOA.split();
        let rx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
        let tx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
        can.assign_pins((tx, rx), &mut afio.mapr);

        bxcan::Can::builder(can)
            .set_bit_timing(0x001c_0003)
            .leave_disabled()
    };

    // APB1 (PCLK1): 8MHz, Bit rate: 125kBit/s, Sample Point 87.5%
    // Value was calculated with http://www.bittiming.can-wiki.info/
    can1.modify_config().set_bit_timing(0x001c_0003);

    // Configure filters so that can frames can be received.
    let mut filters = can1.modify_filters();
    filters.enable_bank(0, Mask32::accept_all());

    #[cfg(feature = "connectivity")]
        let _can2 = {
        let can = Can::new(dp.CAN2);

        let mut gpiob = dp.GPIOB.split();
        let rx = gpiob.pb5.into_floating_input(&mut gpiob.crl);
        let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
        can.assign_pins((tx, rx), &mut afio.mapr);

        let mut can2 = bxcan::Can::new(can);

        // APB1 (PCLK1): 8MHz, Bit rate: 125kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        can2.modify_config().set_bit_timing(0x001c_0003);

        // A total of 28 filters are shared between the two CAN instances.
        // Split them equally between CAN1 and CAN2.
        let mut slave_filters = filters.set_split(14).slave_filters();
        slave_filters.enable_bank(14, Mask32::accept_all());
        can2
    };

    // Drop filters to leave filter configuraiton mode.
    drop(filters);

    // Select the interface.
    let mut can = &mut can1;
    //let mut can = _can2;

    let mut gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // Split the peripheral into transmitter and receiver parts.
    block!(can.enable_non_blocking()).unwrap();
    //Mutex Configuration
    // Store the CAN in the mutex, moving it.
    inter::free(|cs| CAN_M.borrow(cs).replace(Some(dp.CAN1)));
    // We can no longer use `can` or `can1`, and instead have to
    // access it via the mutex.

    //Enable interrupts for CAN RX0 (FIFO 0, see RM8001) after MUtex
    can.enable_interrupt(Fifo0MessagePending);

    //NVIC Enable
    unsafe{NVIC::unmask(Interrupt::USB_LP_CAN_RX0);}

    //New Delay with timer
    let mut timer = Timer::syst(cp.SYST, &clocks).counter_hz();
    timer.start(1.Hz()).unwrap();

    //Send data
    let data1 = Frame::new_data(StandardId::new(5_u16).unwrap(),[1_u8, 1_u8 ,1_u8, 1_u8, 1_u8, 1_u8, 1_u8, 1_u8]);
    let data_off1 = Frame::new_data(StandardId::new(5_u16).unwrap(),[1_u8, 0_u8, 0_u8, 0_u8, 0_u8, 0_u8, 0_u8, 0_u8]);
    let data2 = Frame::new_data(StandardId::new(5_u16).unwrap(),[2_u8, 1_u8 ,1_u8, 1_u8, 1_u8, 1_u8, 1_u8, 1_u8]);
    let data_off2 = Frame::new_data(StandardId::new(5_16).unwrap(), [2_u8, 0_u8, 0_u8, 0_u8, 0_u8, 0_u8, 0_u8, 0_u8]);

    //hprintln!("Debut");

        loop{

            //LED

            led.set_high();
            block!(timer.wait()).unwrap();
            led.set_low();
            block!(timer.wait()).unwrap();

            //Send CODE
/*
            block!(can.transmit(&data1)).unwrap();

            block!(timer.wait()).unwrap();
            //hprintln!("Sent");
            block!(can.transmit(&data2)).unwrap();
            //Wait 1 second
            block!(timer.wait()).unwrap();

            block!(can.transmit(&data_off1)).unwrap();
            //hprintln!("Sent2");
            block!(timer.wait()).unwrap();
            block!(can.transmit(&data_off2)).unwrap();
            //Wait 1 second
            block!(timer.wait()).unwrap();
*/
            //Receive CODE
            //ID recognition

/*
            //Recieve message
            match block!(can.receive()) {
                Ok(v) => {
                    //hprintln!("Read");
                    let read = v.data().unwrap().as_ref();
                    //Check ID = 1
                    //hprintln!("ID = {:?}", v.data().unwrap());
                    if read[0] == 2{

                        if  read[2] == 1 {
                            //led.set_high();
                            hprintln!("HIGH");
                        }
                        else if read[2] == 0 {
                            //led.set_low();
                            hprintln!("LOW");
                        } else {
                            hprintln!("Unknown Command");
                        }
                    } else {
                        hprintln!("NOT ME");
                    }

                }
                Err(e) => {
                    hprintln!("err",);
                }
            }
*/

        }


}
