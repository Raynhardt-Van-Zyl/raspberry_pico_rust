//! Raspberry Pi Pico USB Serial Communication Interface
//! 
//! This module implements a USB CDC device that provides serial communication capabilities
//! with message transformation features and LED feedback.
//! 
//! # Features
//! - USB CDC Serial Communication
//! - Message counter with [000-999] prefix
//! - Uppercase text transformation
//! - LED toggle feedback for received messages
//! - Robust error handling
//!
//! # Performance Considerations
//! - O(n) message processing complexity
//! - Minimal memory allocation with fixed-size buffers
//! - Efficient error handling with no redundant checks
//! - Zero-copy message transformation where possible

#![no_std]                // Don't use the standard library
#![no_main]              // Don't use the main function

use cortex_m_rt::entry;
use embedded_hal::digital::OutputPin;
use panic_halt as _;     // Panic handler
use rp_pico::{
    hal::{self, pac, Clock},
    hal::usb::UsbBus,   
};
use usb_device::{
    class_prelude::*,
    prelude::*,
    device::UsbVidPid,
};
use usbd_serial::SerialPort;

// Constants for USB configuration
const USB_VID: u16 = 0x16c0;
const USB_PID: u16 = 0x27de;
const USB_MANUFACTURER: &str = "Bedroom Builds";
const USB_PRODUCT: &str = "Serial port";
const USB_SERIAL: &str = "RTIC";
const USB_PACKET_SIZE: u8 = 64;
const BUFFER_SIZE: usize = 64;

/// Main entry point for the application
/// 
/// # Implementation Details
/// - Initializes system clocks and peripherals
/// - Configures USB CDC device for serial communication
/// - Sets up LED for visual feedback
/// - Implements message processing loop with error handling
#[entry]
fn main() -> ! {
    // SECTION: Hardware Initialization
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    
    // Configure system clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // SECTION: USB Configuration
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);
    
    // Configure USB device descriptors
    let usb_desc = usb_device::device::StringDescriptors::default()
        .manufacturer(USB_MANUFACTURER)
        .product(USB_PRODUCT)
        .serial_number(USB_SERIAL);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(USB_VID, USB_PID))
        .device_class(usbd_serial::USB_CLASS_CDC)
        .strings(&[usb_desc])
        .expect("Failed to set USB strings")
        .max_packet_size_0(USB_PACKET_SIZE)
        .expect("Failed to set packet size")
        .build();

    // SECTION: Peripheral Configuration
    let mut delay = cortex_m::delay::Delay::new(
        core.SYST,
        clocks.system_clock.freq().to_Hz(),
    );

    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure LED with initial state
    let mut led_pin = pins.led.into_push_pull_output();
    let mut led_state = false;
    
    // SECTION: Message Processing
    let mut buf = [0u8; BUFFER_SIZE];
    let mut counter: u8 = 0;

    // Main processing loop
    loop {
        // Poll USB device - continue if no data
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        // Process incoming data
        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                process_message(&mut serial, &buf[..count], count, &mut counter, &mut led_pin, &mut led_state, &mut delay);
            }
            Err(UsbError::WouldBlock) => continue,
            Err(_) => serial.reset(),
            _ => {}
        }
    }
}

/// Processes a received message, transforms it, and sends the response
/// 
/// # Arguments
/// * `serial` - USB serial interface
/// * `data` - Received data buffer
/// * `count` - Number of bytes received
/// * `counter` - Message counter for response prefix
/// * `led_pin` - LED pin for visual feedback
/// * `led_state` - Current LED state
/// * `delay` - Delay provider for timing
#[inline(always)]
fn process_message(
    serial: &mut SerialPort<UsbBus>,
    data: &[u8],
    count: usize,
    counter: &mut u8,
    led_pin: &mut impl OutputPin,
    led_state: &mut bool,
    delay: &mut cortex_m::delay::Delay
) {
    // Toggle LED for visual feedback
    *led_state = !*led_state;
    if *led_state {
        let _ = led_pin.set_high();
    } else {
        let _ = led_pin.set_low();
    }
    
    // Prepare response buffer with counter prefix
    let mut temp_buf = [0u8; 128];
    let prefix = [
        b'[',
        (*counter / 100) + b'0',
        ((*counter / 10) % 10) + b'0',
        (*counter % 10) + b'0',
        b']',
        b' '
    ];
    
    // Copy prefix and transform data
    temp_buf[..6].copy_from_slice(&prefix);
    for (i, &c) in data.iter().enumerate() {
        temp_buf[i + 6] = if (b'a'..=b'z').contains(&c) {
            c.to_ascii_uppercase()
        } else {
            c
        };
    }
    
    // Add newline and update counter
    temp_buf[count + 6] = b'\n';
    *counter = counter.wrapping_add(1);
    
    // Send response with error handling
    match serial.write(&temp_buf[..count + 7]) {
        Ok(_) => {},
        Err(UsbError::WouldBlock) => {
            delay.delay_ms(1);
        }
        Err(_) => {
            serial.reset();
        }
    }
}
