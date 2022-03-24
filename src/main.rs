// src/main.rs

// std and main are not available for bare metal software
#![no_std]
#![no_main]

mod modules;

// use core::ptr::read;
use cortex_m_rt::entry; // The runtime
//use embedded_hal::digital::v2::OutputPin; // the `set_high/low`function
use stm32f1xx_hal::{delay::Delay, pac, prelude::*}; // STM32F1 specific functions
#[allow(unused_imports)]
use panic_halt;
use stm32f1xx_hal::rcc::Rcc;
use stm32f1xx_hal::pac::Peripherals;
use stm32f1xx_hal::serial::{Config, Serial};

use nb::block;
use cortex_m_semihosting::hprintln;
extern crate drs_0x01;
use drs_0x01::{Servo, Rotation, JogMode, JogColor, WritableEEPAddr, ReadableEEPAddr, WritableRamAddr, ReadableRamAddr};
use drs_0x01::builder::{HerkulexMessage, MessageBuilder};
use drs_0x01::reader::ACKReader;
//use stm32f1xx_hal::pac::ethernet_mac::macvlantr::VLANTC_W;

// const x = ;

// This marks the entrypoint of our application. The cortex_m_rt creates some
// startup code before this, but we don't need to worry about this
#[entry]
fn main() -> ! {

    //
    // CONFIGURATION STM32
    //

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
        Config::default().baudrate(115200.bps()), // baud rate defined in herkulex doc : 115200
        clocks_serial.clone(),
        &mut rcc.apb2,
    );

    // separate into tx and rx channels
    let (mut tx, mut rx) = serial.split();

    let mut delay = Delay::new(cp.SYST, clocks_serial);

    let mut reader = ACKReader::new();

    //
    // END OF CONFIGURATION
    //


    //
    // TO DO LIST
    // X Set ID RAM
    // X Set ID EEPROM
    // X Set ID EEPROM with Paul Library
    // X Get ID RAM
    // X Get ID EEPROM
    // X Make functions
    // @TODO Add interrupts instead of blocking with rx
    // @TODO Generalize set/get for other properties
    // @TODO Test this module
    // @TODO Make documentation
    // @TODO Make it modular to use any TX/RX
    //


    //
    // TESTS
    //

    // Check https://ferrous-systems.com/blog/defmt-test-hal/ to test


    // https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
    // https://github.com/cesarvandevelde/HerkulexServo/blob/master/    Herkulex C++
    // END OF TESTS
    //



    //
    // FUNCTIONS
    //

    ///
    /// Test the function to send messages with TX
    ///
    fn test_sending_message(tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        // Test if the function sendMessage correctly works

        // Make all servos turn for 1 seconds and then stop
        let id = 0xFE; // Broadcast
        let servo = Servo::new(id);
        let msg = servo.set_speed(512, Rotation::Clockwise); // Half speed clockwise
        send_message(msg, tx);

        delay.delay_ms(1_000_u16); // Wait 1 sec

        let msg = servo.set_speed(0, Rotation::Clockwise); // No speed clockwise
        send_message(msg, tx);

        delay.delay_ms(1_000_u16);

    }

    fn test_change_id(tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        let id: u8 = 0x01; // broadcast
        let nextId = 0xF0;
        change_id(id, nextId, tx, delay);
        delay.delay_ms(100_u16);
        set_speed(1, 500, Rotation::Clockwise,tx);
        delay.delay_ms(1_000_u16);
        set_speed(1, 0, Rotation::Clockwise,tx);
        delay.delay_ms(1_000_u16);
    }

    fn test_ids(tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        for i in 238..242 {
            hprintln!("begin id: {:?}", i);
            set_speed(i, 1000, Rotation::Clockwise, tx);
            delay.delay_ms(300_u16);
            set_speed(i, 0, Rotation::Clockwise, tx);
            delay.delay_ms(100_u16);
            hprintln!("end id: {:?}", i);
        }
    }

    ///
    /// Test the function reading with TX
    ///
    fn test_read(rx: &mut stm32f1xx_hal::serial::Rx<stm32f1xx_hal::stm32::USART1>, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        // let id_test = 0x41;
        let servo = Servo::new(get_id_ram(tx,rx));
        // set_ack_policy_ram(0x01,1, tx);
        delay.delay_ms(100_u16);

        let _message = servo.stat();
        // let _message = MessageBuilder::new_with_id(1).stat().build(); // It works
        let getId = servo.ram_request(ReadableRamAddr::ID);
        let getTemperature = servo.ram_request(ReadableRamAddr::Temperature);

        send_message(getTemperature, tx);
        let answ = read_message(rx);


        send_message(getId, tx);
        let answ2 = read_message(rx);

        hprintln!("Temperature : {:?}", answ[9]);
        hprintln!("Id : {:?}", answ2[9]);

        let id_ans = get_id_eeprom(tx, rx);
        hprintln!("Id_f : {:?}", id_ans);

        // read_message(rx, delay);
        delay.delay_ms(10_u16);


    }

    ///
    /// Send message in through tx
    /// @param msg   : Herkulex message which is an array containing the bytes to send
    /// @param tx    : Tx protocol
    ///
    fn send_message(msg : HerkulexMessage, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>) {
        for b in &msg {
            block!(tx.write(*b)).ok();
        }
    }

    ///
    /// Read message through rx
    /// ONLY Display info
    /// @returns tuple of the array and the size of the packet
    ///
    fn read_message(rx: &mut stm32f1xx_hal::serial::Rx<stm32f1xx_hal::stm32::USART1>) -> [u8;20] {
        let mut received_message:[u8; 20] = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];

        let mut i = 0;
        let mut pSize = 3; // Read till you know the packet size and then refresh it
        while i < pSize {
            let res = block!(rx.read());
            match res {
                Ok(vr) => {
                    received_message[i] = vr;
                    if i == 2 {
                        pSize = vr as usize; // Refresh packetSize
                    }
                },
                Err(e) => {
                    hprintln!("{:?}",e);
                }
            }
            i+=1;
        }

        // Display packet received
        // hprintln!("Received packet : {:?}", &received_message[0..pSize]);

        received_message

    }


    ///
    /// Init a servo by rebooting, cleaning errors and setting torque on
    /// @param id    : the id of the targeted servo
    ///
    fn init(id : u8, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        let servo = Servo::new(id);

        //? Reboot the servo needed ?
        send_message(servo.reboot(), tx);
        delay.delay_ms(1_00_u16);

        // Clear errors
        send_message(servo.clear_errors(), tx);
        delay.delay_ms(100_u16);

        // Enable torque
        // It is compulsory to make the servo turn
        send_message(servo.enable_torque(), tx);
        delay.delay_ms(100_u16);

        //set_ack_policy_ram(id, 2, tx);
    }

    ///
    /// Change the id of servo in the memory
    ///
    fn change_id(currentid: u8, newid: u8, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        let servo = Servo::new(currentid);
        let writeid1 = servo.ram_write(WritableRamAddr::ID(newid));
        send_message(writeid1, tx);
        delay.delay_ms(100_u16);
    }


    ///
    /// Change the id of servo in the EEPROM
    /// Same msg as Paul's lib
    /// @param currentid : the id of the servo you want to change
    /// @param newid     : the id you want to set to the servo
    ///
    fn change_id_eeprom(current_id: u8, new_id: u8, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        if(current_id != new_id) {

            let servo = Servo::new(current_id);
            let msg = servo.eep_write(WritableEEPAddr::ID(new_id));
            hprintln!("Change id from {:?} to {:?}", current_id, new_id).ok();

            send_message(msg,tx);

            let servo = Servo::new(new_id);

            // Restart the motor
            send_message(servo.reboot(), tx);

            // Clear errors
            let clear_error_msg = servo.clear_errors();
            send_message(clear_error_msg,tx);

            // Enable torque;
            let torque_on_msg = servo.enable_torque();
            send_message(torque_on_msg, tx);

            delay.delay_ms(100_u16); // It does not work without it, idk if we can lower the delay
        }

    }

    ///
    /// Get id of the servo in RAM
    ///
    fn get_id_ram(tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, rx: &mut stm32f1xx_hal::serial::Rx<stm32f1xx_hal::stm32::USART1>) -> u8 {
        let broadcast = 0xFE;
        let servo = Servo::new(broadcast);
        // let msg = servo.eep_request(ReadableEEPAddr::ID);
        let msg = servo.ram_request(ReadableRamAddr::ID);
        let msg = servo.stat();
        send_message(msg, tx);

        let ans = read_message(rx);
        ans[3]
    }

    ///
    /// Get id of the servo in EEPROM
    ///
    fn get_id_eeprom(tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, rx: &mut stm32f1xx_hal::serial::Rx<stm32f1xx_hal::stm32::USART1>) -> u8 {
        let servo = Servo::new(get_id_ram(tx,rx)); // Doesn't answer to broadcast;

        let msg = servo.eep_request(ReadableEEPAddr::ID);
        send_message(msg, tx);

        let ans = read_message(rx);
        ans[9]
    }

    ///
    /// Set the speed of a servo
    /// @param id        : the id of the servo you want to set the speed to
    /// @param speed     : the speed from 0 to 1000
    /// @param rotation  : the rotation Clockwise or counterclockwise
    ///
    fn set_speed(id : u8, speed : u16, rotation : Rotation, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>) {
        let servo = Servo::new(id);
        let msg = servo.set_speed(speed, rotation);
        send_message(msg, tx);
    }



    ///
    /// Set ACK Policy of the servo in RAM
    /// @param id        (u8)    - id of the servo (Broadcast id does not work)
    /// @param policy    (u8)    - AckPolicy (0,1,2)
    ///
    fn set_ack_policy_ram(id : u8, policy : u8, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>) {
        let servo = Servo::new(id);
        let msg = servo.ram_write(WritableRamAddr::AckPolicy(policy));
        send_message(msg ,tx);
    }

    //
    // END OF FUNCTIONS
    //






    //
    // INIT SERVOS
    //

    // Servo with Broadcast id
    let servo_broadcast = Servo::new(0xFE);
    init(servo_broadcast.id(), &mut tx, &mut delay);

    let servo = Servo::new(get_id_ram(&mut tx, &mut rx));
    let new_id = 0x28;
    change_id_eeprom(servo.id(), new_id, &mut tx, &mut delay);

    //
    // END OF INIT SERVOS
    //


    // Servo branche, ID = 240;


    loop {
        test_read( &mut rx, &mut tx, &mut delay);
        delay.delay_ms(100_u16);
    }
}
