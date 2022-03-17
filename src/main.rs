// src/main.rs

// std and main are not available for bare metal software
#![no_std]
#![no_main]

mod modules;

use core::ptr::read;
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
    // @TODO Set ID EEPROM with Paul Library
    // @TODO Get ID RAM
    // @TODO Get ID EEPROM
    // X Make functions
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
    /// This function is here to try and test random things
    /// @TODO Remove this function when it became useless
    ///
    fn test_function(tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        // Test if the function sendMessage correctly works
        // If it works then @TODO Update all other functions to use sendMessage function


        // Make all servos turn for 1 seconds and then stop
        let id = 0x01; // Broadcast
        let servo = Servo::new(id);
        let msg = servo.set_speed(512, Rotation::Clockwise); // Half speed clockwise
        send_message(msg, tx);

        delay.delay_ms(1_000_u16); // Wait 1 sec

        let msg = servo.set_speed(0, Rotation::Clockwise); // No speed clockwise
        send_message(msg, tx);

        delay.delay_ms(1_000_u16);

    }

    fn test_function2(tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        let id: u8 = 0x01; // broadcast
        let nextId = 0xF0;
        change_id(id, nextId, tx, delay);
        delay.delay_ms(100_u16);
        set_speed(1, 500, Rotation::Clockwise,tx);
        delay.delay_ms(1_000_u16);
        set_speed(1, 0, Rotation::Clockwise,tx);
        delay.delay_ms(1_000_u16);

        // let servo = Servo::new(nextId);
        // send_message(servo.reboot(), tx);
        // delay.delay_ms(100_u16);
        // init(nextId, tx, delay);
    }

    fn tester_ids(tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        for i in 238..242 {
            hprintln!("begin id: {:?}", i);
            set_speed(i, 1000, Rotation::Clockwise, tx);
            delay.delay_ms(300_u16);
            set_speed(i, 0, Rotation::Clockwise, tx);
            delay.delay_ms(100_u16);
            hprintln!("end id: {:?}", i);
        }
    }

    fn tester_read(reader : &mut ACKReader ,rx: &mut stm32f1xx_hal::serial::Rx<stm32f1xx_hal::stm32::USART1>, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        change_id(0xFE, 0x01, tx, delay);
        let servo = Servo::new(0x01);
        set_ack_policy_ram(0x01,1, tx);
        delay.delay_ms(100_u16);

        let _message = servo.stat();
        // let _message = MessageBuilder::new_with_id(1).stat().build(); // It works
        let getId = servo.ram_request(ReadableRamAddr::ID); // No answer yet => check ack policy
        let getTemperature = servo.ram_request(ReadableRamAddr::Temperature);

        // let mut received_message:[u8; 10] = [0,0,0,0,0,0,0,0,0,0];

        // hprintln!("{:?}", getId);

        // send_message(getId, tx);
        // send_message(_message, tx);
        //
        // // It works
        // read_bytes(rx, &mut received_message, 0);
        // hprintln!("{:?}", received_message);

        // let _message = servo.stat();
        // send_message(_message, tx);
        // send_message(getId, tx);
        send_message(getTemperature, tx);



        // delay.delay_ms(1_u16); // No delay or overrun



        let mut received_message:[u8; 15] = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
        for i in 0..3 {
            let res = block!(rx.read());
            match res {
                Ok(vr) => {
                    received_message[i] = vr;
                },
                Err(e) => {
                    hprintln!("{:?}",e);
                },
                _ => {

                }
            }
        }
        let pSize:usize = received_message[2] as usize;
        for i in 3..pSize {
            let res = block!(rx.read());
            match res {
                Ok(vr) => {
                    received_message[i] = vr;
                },
                Err(e) => {
                    hprintln!("{:?}",e);
                },
                _ => {

                }
            }
        }



        // read_bytes(rx, &mut received_message, 0);
        hprintln!("{:?}", &received_message[0..pSize]);


        // read_message(rx, delay);
        delay.delay_ms(10_u16);


    }

    ///
    /// It works
    fn read_bytes(rx: &mut stm32f1xx_hal::serial::Rx<stm32f1xx_hal::stm32::USART1>, received_message : &mut [u8;10], i :usize) {
        // let res = block!(rx.read());
        match block!(rx.read()) {
                Ok(vr) => {
                    received_message[i] = vr;
                    read_bytes(rx, received_message, i+1);
                },
                Err(e) => {
                    hprintln!("{:?}",e);
                },
                _ => {

                }
        }
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
    /// @returns ??
    /// @TODO Return the result
    /// @TODO Check if it works
    ///
    fn read_message(rx: &mut stm32f1xx_hal::serial::Rx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) -> u8 {
        let mut reader = ACKReader::new();

        let res = block!(rx.read());
        match res {
            Ok(vr) => {
                hprintln!("{:?}", vr);
            },
            Err(e) => {
                hprintln!("{:?}",e);
            }
        }
        // hprintln!("id: {:?}", res).unwrap();

        // let msg_nb = reader.available_messages();
        // hprintln!("{:?}",msg_nb).ok();
        //
        // let received_message = [0u8];
        // reader.parse(&received_message);
        // match reader.pop_ack_packet() {
        //     Some(pk) => {
        //         hprintln!("{:?}",pk).ok();
        //     },
        //     _ => {
        //         hprintln!("pass").ok();
        //     }
        // };

        let msg : u8 = 42;
        msg

    }


    ///
    /// Init a servo by cleaning errors and setting torque on
    /// @param id    : the id of the targeted servo
    /// @TODO Check if we need to reboot the servo
    ///
    fn init(id : u8, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        let servo = Servo::new(id);

        //? Reboot the servo ?
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
    /// TODO : Check if it works
    ///
    fn change_id_eeprom(current_id: u8, new_id: u8, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        let packet_size = 0x0A;
        let packet_id = current_id;
        let cmd = 0x01;
        let addr = 0x06;
        let length = 0x01;
        let packet_new_id = new_id;

        // Checksum1
        let mut ck1 = 0;
        ck1 ^= packet_size;
        ck1 ^= packet_id;
        ck1 ^= cmd;
        ck1 ^= addr;
        ck1 ^= length;
        ck1 ^= packet_new_id;
        ck1 &= 0xFE;

        // Checksum2
        let ck2 = (!ck1) & 0xFE;

        // Packet Header, Packet Header, Packet Size, Servo id, Command, Chksum1, chksum2, Address, Length, Value
        let changeid_msg: [u8; 10] = [0xFF, 0xFF, packet_size, packet_id, cmd, ck1, ck2, addr, length, packet_new_id];
        let servo = Servo::new(current_id);
        let msg = servo.eep_write(WritableEEPAddr::ID(new_id));
        hprintln!("{:?} to {:?}", changeid_msg, msg).ok();
        hprintln!("Change id from {:?} to {:?}", current_id, new_id).ok();


        for b in &changeid_msg {
            block!(tx.write(*b)).ok();
        }
        delay.delay_ms(10_00_u16);

        let servo = Servo::new(new_id);

        // Restart the motor
        let reboot_msg = servo.reboot();
        for b in &reboot_msg {
            block!(tx.write(*b)).ok();
        }
        delay.delay_ms(5_00_u16);

        // Clear errors
        let clear_error_msg = servo.clear_errors();
        delay.delay_ms(1_00_u16);
        for b in &clear_error_msg {
            block!(tx.write(*b)).ok();
        }
        // Enable torque;
        let torque_on_msg = servo.enable_torque();
        delay.delay_ms(10_u16);
        for b in &torque_on_msg {
            block!(tx.write(*b)).ok();
        }
        delay.delay_ms(100_u16);
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
        for b in &msg {
            block!(tx.write(*b)).ok();
        }
    }


    ///
    /// Get id of the servo in RAM
    /// @TODO Get the message to return it
    /// @TODO Check if it works
    ///
    fn get_id_ram(tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, rx: &mut stm32f1xx_hal::serial::Rx<stm32f1xx_hal::stm32::USART1> ,delay: &mut stm32f1xx_hal::delay::Delay) -> u8 {
        let broadcast = 0xFE;
        let servo = Servo::new(broadcast);
        let msg = servo.eep_request(ReadableEEPAddr::ID);
        send_message(msg, tx);

        delay.delay_ms(10_u16);

        read_message(rx, delay) // Returns 42 for now @TODO Check the result here
    }

    ///
    /// Set ACK Policy of the servo in RAM
    /// @param id        (u8)    - id of the servo (Broadcast id does not work)
    /// @param policy    (u8)    - AckPolicy (0,1,2)
    /// @TODO Check if it works
    ///
    fn set_ack_policy_ram(id : u8, policy : u8, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>) {
        let servo = Servo::new(id);
        let msg = servo.ram_write(WritableRamAddr::AckPolicy(policy));
        for b in &msg {
            block!(tx.write(*b)).ok();
        }
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

    //
    // END OF INIT SERVOS
    //


    // test_function2(&mut tx, &mut delay);


    // Servo branche, ID = 240;


    loop {
        tester_read(&mut reader, &mut rx, &mut tx, &mut delay);
        delay.delay_ms(100_u16);
        // change_id_eeprom(240, 1, &mut tx, &mut delay);

        // tester_ids(&mut tx, &mut delay);
    }
}
