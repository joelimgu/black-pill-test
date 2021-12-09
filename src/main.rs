// src/main.rs

// std and main are not available for bare metal software
#![no_std]
#![no_main]

mod modules;

use cortex_m_rt::entry; // The runtime
use embedded_hal::digital::v2::OutputPin; // the `set_high/low`function
use stm32f1xx_hal::{delay::Delay, pac, prelude::*}; // STM32F1 specific functions
#[allow(unused_imports)]
use panic_halt;
use stm32f1xx_hal::rcc::Rcc;
use stm32f1xx_hal::pac::Peripherals;
use stm32f1xx_hal::serial::{Config, Serial, Tx};

use nb::block;
use cortex_m_semihosting::hprintln;

extern crate drs_0x01;
use drs_0x01::{Servo, Rotation, JogMode, JogColor, WritableEEPAddr, ReadableEEPAddr};
use drs_0x01::builder::{HerkulexMessage, MessageBuilder};
use drs_0x01::reader::ACKReader;
use stm32f1xx_hal::pac::ethernet_mac::macvlantr::VLANTC_W;

// This marks the entrypoint of our application. The cortex_m_rt creates some
// startup code before this, but we don't need to worry about this
#[entry]
fn main() -> ! {

    // Get handles to the hardware objects. These functions can only be called
    // once, so that the borrowchecker can ensure you don't reconfigure
    // something by accident.
    let dp: Peripherals = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // GPIO pins on the STM32F1 must be driven by the APB2 peripheral clock.
    // This must be enabled first. The HAL provides some abstractions for

    // us: First get a handle to the RCC peripheral:
    let mut rcc: Rcc = dp.RCC.constrain();
    // Now we have access to the RCC's registers. The GPIOC can be enabled in
    // RCC_APB2ENR (Prog. Ref. Manual 8.3.7), therefore we must pass this
    // register to the `split` function.
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    // This gives us an exclusive handle to the GPIOC peripheral. To get the
    // handle to a single pin, we need to configure the pin first. Pin C13
    // is usually connected to the Bluepills onboard LED.

    let mut flash = dp.FLASH.constrain();
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

    // let sys_clock = rcc.cfgr.sysclk(8.mhz()).freeze(&mut flash.acr);
    let clocks_serial = rcc.cfgr.freeze(&mut flash.acr);

    // USART1 on Pins A9 and A10
    let pin_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let pin_rx = gpioa.pa10;

    let serial = Serial::usart1(
        dp.USART1,
        (pin_tx, pin_rx),
        &mut afio.mapr,
        Config::default().baudrate(115200.bps()), // baud rate defined in herkulex doc
        clocks_serial.clone(),
        &mut rcc.apb2,
    );

    // separate into tx and rx channels
    let (mut tx,  mut rx) = serial.split();

    let mut delay = Delay::new(cp.SYST, clocks_serial);
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    let mut servo = Servo::new(0xFE);
    let message = servo.set_speed(512, Rotation::Clockwise);

    let reboot_msg: HerkulexMessage = servo.reboot();

    let clear_error_msg = servo.clear_errors();

    let torque_on_msg = servo.enable_torque();

    led.set_low().ok();
    delay.delay_ms(1_00_u16);
    for b in &reboot_msg {
        block!(tx.write(*b)).ok();
    }
    delay.delay_ms(5_00_u16);
    delay.delay_ms(1_00_u16);
    for b in &clear_error_msg {
        block!(tx.write(*b)).ok();
    }
    // delay.delay_ms(10_u16);
    // for b in &ACK_msg {
    //     block!(tx.write(*b));
    // }
    delay.delay_ms(10_u16);
    for b in &torque_on_msg {
        block!(tx.write(*b)).ok();
    }
    delay.delay_ms(10_u16);

    // let message = MessageBuilder::new().id(0x00).s_jog(60, JogMode::Continuous{speed: 512, rotation: Rotation::CounterClockwise }, JogColor::Green, 0x01 ).build();
    // let message = MessageBuilder::new_with_id(0xFE).write_eep(WritableEEPAddr::ID(0x05)).build();
    // let message = MessageBuilder::new_with_id(0xFE).read_eep(ReadableEEPAddr::ID, 4).build();

    let id_servo = 5;

    let message2 = MessageBuilder::new_with_id(0xFE).stat().build();

    servo.set_id(id_servo);

    /*
    for b in &changeID_msg {
        block!(tx.write(*b)).ok();
    }
    delay.delay_ms(1_000_u16);


     */

    delay.delay_ms(1_00_u16);
    for b in &reboot_msg {
        block!(tx.write(*b)).ok();
    }
    delay.delay_ms(5_00_u16);
    delay.delay_ms(1_00_u16);
    for b in &clear_error_msg {
        block!(tx.write(*b)).ok();
    }
    delay.delay_ms(10_u16);
    for b in &torque_on_msg {
        block!(tx.write(*b)).ok();
    }
    delay.delay_ms(10_u16);




    let mut servo_id = 0x0D;

    let mut ID_Old = 0x0A;
    let mut ID_New = servo_id;

    loop {
        if servo_id > 0x0D {
            servo_id = 0x0A
        }
        //
        // set ID, from OLD to NEW
        //


        // Parameters
        ID_Old = ID_Old;
        ID_New = servo_id;


        let pSize = 0x0A;
        let mut pID = ID_Old;
        let cmd = 0x01;
        let addr = 0x06;
        let length = 0x01;
        let mut sNew = ID_New;

        // Checksum1
        let mut ck1 = 0;
        ck1 = ck1 ^ 0x0A;
        ck1 = ck1 ^ ID_Old;
        ck1 = ck1 ^ 0x01;
        ck1 = ck1 ^ addr;
        ck1 = ck1 ^ length;
        ck1 = ck1 ^ sNew;
        ck1 = ck1&0xFE;

        // Checksum2
        let mut ck2 = (!ck1)&0xFE;

        // Packet Header, Packet Header, Packet Size, Servo ID, Command, Chksum1, chksum2, Address, Length, Value
        let mut changeID_msg: [u8; 10] = [0xFF, 0xFF, pSize, pID, cmd, ck1, ck2, addr, length, ID_New];

        hprintln!("Change ID from {:?} to {:?}", ID_Old, ID_New);

        for b in &changeID_msg {
            block!(tx.write(*b)).ok();
        }

        // Restart the motor
        delay.delay_ms(10_00_u16);
        for b in &reboot_msg {
            block!(tx.write(*b)).ok();
        }
        delay.delay_ms(5_00_u16);

        // Clear errors
        delay.delay_ms(1_00_u16);
        for b in &clear_error_msg {
            block!(tx.write(*b)).ok();
        }
        // Enable torque;
        delay.delay_ms(10_u16);
        for b in &torque_on_msg {
            block!(tx.write(*b)).ok();
        }
        delay.delay_ms(100_u16);

        for i in 9..0x0E{
            hprintln!("Trying servo with id: {}", i);
            let servo = Servo::new(i);

            let mut message = servo.set_speed(512, Rotation::Clockwise);

            for b in &message {
                block!(tx.write(*b)).ok();
            }

            delay.delay_ms(1_000_u16);
            message = servo.set_speed(0, Rotation::Clockwise);
            for b in &message {
                block!(tx.write(*b)).ok();
            }
            delay.delay_ms(1_000_u16);
        }
        ID_Old = servo_id;
        servo_id += 1;


        /*
        let _r = block!(rx.read()).unwrap();
        delay.delay_ms(1_00_u16);
        hprintln!("ID: {:?}", message).unwrap();
        hprintln!("After read");
        */

        /*
        let mut reader = ACKReader::new();
        for b in &message {
            block!(tx.write(*b)).ok();
        }

        delay.delay_ms(1_00_u16);

        /*
        let res = block!(rx.read()).unwrap();
        delay.delay_ms(1_00_u16);
        hprintln!("ID: {:?}", res).unwrap();
        */

        let received_message = [0u8];
        reader.parse(&received_message);
        match reader.pop_ack_packet() {
            Some(pk) => {
                hprintln!("{:?}",pk).ok();
            },
            _ => {
                hprintln!("pass").ok();
            }
        }

         */
    }


    // cortex_m_semihosting::hprintln!("starting...").ok();
    // modules::black_pill::func()

}
