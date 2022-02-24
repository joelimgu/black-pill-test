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
use drs_0x01::{Servo, Rotation, JogMode, JogColor, WritableEEPAddr, ReadableEEPAddr, WritableRamAddr, ReadableRamAddr};
use drs_0x01::builder::{HerkulexMessage, MessageBuilder};
use drs_0x01::reader::ACKReader;
use stm32f1xx_hal::pac::ethernet_mac::macvlantr::VLANTC_W;

// This marks the entrypoint of our application. The cortex_m_rt creates some
// startup code before this, but we don't need to worry about this
#[entry]
fn main() -> ! {


    ///
    /// CONFIGURATION STM32
    ///

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
    let (mut tx, mut rx) = serial.split();

    let mut delay = Delay::new(cp.SYST, clocks_serial);

    ///
    /// END OF CONFIGURATION
    ///








    ///
    /// FUNCTIONS
    ///


    /**
     * Send message in through tx
     * @param msg   : Herkulex message which is an array containing the bytes to send
     * @param tx    : Tx protocol
     * @TODO Check if it works
     */
    fn sendMessage(msg : HerkulexMessage, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>) {
        for b in &msg {
            block!(tx.write(*b)).ok();
        }
    }

    /**
     * Read message through rx
     * ONLY Display info
     * @TODO Return the result
     * @TODO Check if it works
     */
    fn readMessage(tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, rx: &mut stm32f1xx_hal::serial::Rx<stm32f1xx_hal::stm32::USART1>) {
        let mut reader = ACKReader::new();
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
    }


    /**
     * Init a servo by cleaning errors and setting torque on
     * @param ID    : the ID of the targeted servo
     * @TODO  Check if it works
     * @TODO Check if we need to reboot the servo
     */
    fn init(ID : u8, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        let servo = Servo::new(ID);
        let msg_clear_error = servo.clear_errors();
        let msg_torque_on = servo.enable_torque();

        //? Reboot the servo ?

        // Clear errors
        for b in &msg_clear_error {
            block!(tx.write(*b)).ok();
        }
        delay.delay_ms(1_00_u16);

        // Enable torque
        // It is compulsory to make the servo turn
        for b in &msg_torque_on {
            block!(tx.write(*b)).ok();
        }
        delay.delay_ms(1_00_u16);
    }

    /**
     * Change the ID of servo in the memory
     * @TODO : Check if it works
    **/
    fn changeID(currentID: u8, newID: u8, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        let servoTest = Servo::new(currentID);
        let writeID1 = servoTest.ram_write(WritableRamAddr::ID(newID));
        for b in &writeID1 {
            block!(tx.write(*b)).ok();
        }
        delay.delay_ms(10_00_u16);

        let clear_errors = servoTest.clear_errors();
        for b in &clear_errors {
            block!(tx.write(*b)).ok();
        }

        delay.delay_ms(100_00_u16);

        let enable_torque = servoTest.enable_torque();
        for b in &enable_torque {
            block!(tx.write(*b)).ok();
        }
        delay.delay_ms(10_00_u16);
    }


    /**
     * Change the ID of servo in the EEPROM
     * @param currentID : the ID of the servo you want to change
     * @param newID     : the ID you want to set to the servo
     * TODO : Check if it works
     */
    fn changeIDEEPROM(currentID: u8, newID: u8, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        let pSize = 0x0A;
        let mut pID = currentID;
        let cmd = 0x01;
        let addr = 0x06;
        let length = 0x01;
        let mut sNew = newID;

        // Checksum1
        let mut ck1 = 0;
        ck1 ^= 0x0A;
        ck1 ^= currentID;
        ck1 ^= 0x01;
        ck1 ^= addr;
        ck1 ^= length;
        ck1 ^= sNew;
        ck1 ^= 0xFE;

        // Checksum2
        let mut ck2 = (!ck1) & 0xFE;

        // Packet Header, Packet Header, Packet Size, Servo ID, Command, Chksum1, chksum2, Address, Length, Value
        let mut changeID_msg: [u8; 10] = [0xFF, 0xFF, pSize, pID, cmd, ck1, ck2, addr, length, newID];

        hprintln!("Change ID from {:?} to {:?}", currentID, newID);


        for b in &changeID_msg {
            block!(tx.write(*b)).ok();
        }
        delay.delay_ms(10_00_u16);

        let servo = Servo::new(newID);

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

    /**
     * Set the speed of a servo
     * @param ID        : the ID of the servo you want to set the speed to
     * @param speed     : the speed from 0 to 1000
     * @param rotation  : the rotation Clockwise or counterclockwise
     */
    fn setSpeed(ID : u8, speed : u16, rotation : Rotation, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay ) {
        let servo = Servo::new(ID);
        let msg = servo.set_speed(speed, rotation);
        for b in &msg {
            block!(tx.write(*b)).ok();
        }
    }


    /**
     * Get ID of the servo in RAM
     * @TODO Get the message to return it
     * @TODO Check if it works
     */
    fn getIDRAM(tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, rx: &mut stm32f1xx_hal::serial::Rx<stm32f1xx_hal::stm32::USART1> ,delay: &mut stm32f1xx_hal::delay::Delay) -> u8 {
        let answer : u8 = 31;
        let broadcast = 0xFE;
        let servo = Servo::new(broadcast);
        let msg = servo.eep_request(ReadableEEPAddr::ID);
        sendMessage(msg, tx);

        answer
    }

    /**
     * Set ACK Policy of the servo in RAM
     * @param ID        (u8)    - id of the servo
     * @param policy    (u8)    - AckPolicy (0,1,2)
     * @TODO Check if it works
     */
    fn setAckPolicyRam(ID : u8, policy : u8, tx: &mut stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART1>, delay: &mut stm32f1xx_hal::delay::Delay) {
        let servo = Servo::new(ID);
        let msg = servo.ram_write(WritableRamAddr::AckPolicy(policy));
        for b in &msg {
            block!(tx.write(*b)).ok();
        }
    }

    ///
    /// END OF FUNCTIONS
    ///







    ///
    /// INIT SERVOS
    ///

    // Servo with Broadcast ID
    let mut servo = Servo::new(0xFE);
    init(servo.id(), &mut tx, &mut delay);

    ///
    /// END OF INIT SERVOS
    ///





    loop {
        // Try all IDs from 0 to 10
        // Make the motor with the corresponding ID turn for 500ms and then stop
        for i in 0x00..0x0B{
            hprintln!("Trying servo with id: {}", i);
            setSpeed(i, 512, Rotation::Clockwise,  &mut tx, &mut delay);
            delay.delay_ms(500_u16);

            setSpeed(i, 0, Rotation::Clockwise,  &mut tx, &mut delay);
            delay.delay_ms(200_u16);
        }
    }
}
