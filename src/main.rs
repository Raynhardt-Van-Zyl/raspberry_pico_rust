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

#[entry]
fn main() -> ! {
    // Get access to the device specific peripherals
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    
    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    
    // Configure the clocks
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

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);
    let usb_desc = usb_device::device::StringDescriptors::default()
        .manufacturer("Bedroom Builds")
        .product("Serial port")
        .serial_number("RTIC");

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27de))
        .device_class(usbd_serial::USB_CLASS_CDC)
        .strings(&[usb_desc])
        .expect("Failed to set USB strings")
        .max_packet_size_0(64)
        .expect("Failed to set packet size")
        .build();

    // The delay object lets us wait for specified amounts of time
    let mut delay = cortex_m::delay::Delay::new(
        core.SYST,  // Using the core.SYST we got earlier
        clocks.system_clock.freq().to_Hz(),
    );

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);
    
    // Set the pins to their default state
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure the LED pin (GPIO25)
    let mut led_pin = pins.led.into_push_pull_output();
    let mut led_state = false;  // Track LED state
    
    // Add buffer for USB data and counter
    let mut buf = [0u8; 64];
    let mut counter = 0u8;

    loop {
        // Poll the USB device
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        // Check for new data
        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Toggle LED
                led_state = !led_state;
                if led_state {
                    led_pin.set_high().unwrap();
                } else {
                    led_pin.set_low().unwrap();
                }
                
                // Create a temporary buffer with counter prefix
                let mut temp_buf = [0u8; 128];
                let prefix = [
                    b'[',
                    (counter / 100) + b'0',
                    ((counter / 10) % 10) + b'0',
                    (counter % 10) + b'0',
                    b']',
                    b' '
                ];
                
                // Copy prefix to temp buffer
                temp_buf[..6].copy_from_slice(&prefix);
                
                // Convert and copy main data
                for (i, c) in buf[..count].iter().enumerate() {
                    temp_buf[i + 6] = if (b'a'..=b'z').contains(c) {
                        c.to_ascii_uppercase()
                    } else {
                        *c
                    };
                }
                
                // Add newline
                temp_buf[count + 6] = b'\n';
                
                counter = counter.wrapping_add(1);
                
                // Write the complete buffer
                match serial.write(&temp_buf[..count + 7]) {
                    Ok(_) => {
                        // Removed led_pin.set_low() from here
                    }
                    Err(UsbError::WouldBlock) => {
                        delay.delay_ms(1);
                    }
                    Err(_) => {
                        serial.reset();
                    }
                }
            }
            Err(UsbError::WouldBlock) => {
                continue;
            }
            Err(_) => {
                serial.reset();
            }
            _ => {}
        }
    }
}
