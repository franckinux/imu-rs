#![no_std]
#![no_main]

use core::fmt::Write;
use core::iter::once;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// Some traits we need
// use cortex_m::prelude::*;
use cortex_m;
use fugit::RateExtU32;
use hal::{
    clocks::Clock,
    gpio::{FunctionI2C, Pin, PullUp},
    pac,
};

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

use hal::pio::PIOExt;

use heapless::String;

use shared_bus::BusManagerSimple;
use mpu6050;
use qmc5883l::*;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;


/// Convert a number from `0..=255` to an RGB color triplet.
///
/// The colours are a transition from red, to green, to blue and back to red.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        // No green in this sector - red and blue only
        (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    } else if wheel_pos < 170 {
        // No red in this sector - green and blue only
        wheel_pos -= 85;
        (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    } else {
        // No blue in this sector - red and green only
        wheel_pos -= 170;
        (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    }
}


fn write_serial(ser: &mut SerialPort<hal::usb::UsbBus>, string: &str)  {
    match ser.write(string.as_bytes()) {
        Ok(_count) => {
            // count bytes were written
        },
        Err(UsbError::WouldBlock) => {},  // No data could be written (buffers full)
        Err(_err) => {},  // An error occurred
    };
}


/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, then performs some example
/// SPI transactions, then goes to sleep.
#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // The delay object lets us wait for specified amounts of time (in milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // === I2C ===
    // Configure two pins as being I²C, not GPIO
    let sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio14.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio15.reconfigure();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
    let i2c_bus = BusManagerSimple::new(i2c);

    // === SPI ===

    // === USB ===
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x55aa, 0x0001))
        .manufacturer("franckinux")
        .product("imu-rs")
        .serial_number("00000001")
        .device_class(USB_CLASS_CDC) // from: https://www.usb.org/defined-class-codes
        .build();

    // === RGB led ===
    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        // The onboard NeoPixel is attached to GPIO pin #16 on the Feather RP2040.
        pins.gpio16.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    // === MPU6050 ===
    let mut mpu = mpu6050::Mpu6050::new(i2c_bus.acquire_i2c());
    mpu.init(&mut delay).unwrap();

    // === QMC5883 ===
    let mut qmc = QMC5883L::new(i2c_bus.acquire_i2c()).unwrap();
    qmc.continuous().unwrap();

    // Infinite colour wheel loop
    let mut n: u8 = 128;
    loop {
        delay.delay_ms(10);

        ws.write(brightness(once(wheel(n)), 32)).unwrap();
        n = n.wrapping_add(1);


        if !usb_dev.poll(&mut [&mut serial]) {
            continue
        }

        // get gyro data, scaled with sensitivity
        let gyro = mpu.get_gyro().unwrap();
        // write!(temp_str, "gyro: {:?}\r\n", gyro).unwrap();
        // write_serial(&mut serial, temp_str.as_str());
        // usb_dev.poll(&mut [&mut serial]);

        // get accelerometer data, scaled with sensitivity
        let acc = mpu.get_acc().unwrap();
        // write!(temp_str, "acc: {:?}\r\n", acc).unwrap();
        // write_serial(&mut serial, temp_str.as_str());
        // usb_dev.poll(&mut [&mut serial]);

        let (x, y, z) = qmc.mag().unwrap();
        // write!(temp_str, "mag: {:?}, {:?}, {:?}\r\n", x, y, z).unwrap();
        // write_serial(&mut serial, temp_str.as_str());
        // usb_dev.poll(&mut [&mut serial]);

        if n % 20 == 0 {
            // let mut temp_str: String<512> = String::new();

        }
    }
}

// End of file
