#![no_std]
#![no_main]

// Used frame : ENU
// Rotation order is ZYX

// psi (Ψψ) = yaw = lacet
// theta (Θθ) = pitch = tangage
// phi a(Φφ) = roll = roulis

use core::fmt::Write;
use core::iter::once;

use embedded_hal::timer::CountDown;
use fugit::ExtU32;
use libm;

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
    pio::PIOExt,
};

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

use heapless::String;

use shared_bus::BusManagerSimple;
use mpu6050;
use qmc5883l;

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
    let mut qmc = qmc5883l::QMC5883L::new(i2c_bus.acquire_i2c()).unwrap();
    qmc.continuous().unwrap();

    // Infinite colour wheel loop
    let mut n: u8 = 128;
    let mut count_down = timer.count_down();
    loop {
        count_down.start(10.millis());

        ws.write(brightness(once(wheel(n)), 32)).unwrap();
        n = n.wrapping_add(1);

        // get accelerometer data, scaled with sensitivity
        let acc = mpu.get_acc().unwrap();
        let acc_x = -acc.x as f64;
        let acc_y = -acc.y as f64;
        let acc_z = -acc.z as f64;

        // get gyro data, scaled with sensitivity
        let gyro = mpu.get_gyro().unwrap();
        let gyro_x = gyro.x as f64;
        let gyro_y = gyro.y as f64;
        let gyro_z = gyro.z as f64;

        let (mag_x, mag_y, mag_z) = qmc.mag().unwrap();
        let mag_x = -mag_x as f64;
        let mag_y = -mag_y as f64;
        let mag_z = -mag_z as f64;

        // compute theta and phi from the accelerometer
        let a = libm::sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
        let phi = -libm::asin(acc_y / a);
        let theta = libm::asin(acc_x / a);

        let cp = libm::cos(phi);
        let sp = libm::sin(phi);
        let ct = libm::cos(theta);
        let st = libm::sin(theta);

        // compute tilt compensated theta and phi
        let mag_x_compensated = mag_x * ct + mag_y * sp * st - mag_z * cp * st;
        let mag_y_compensated = mag_y * cp + mag_z * sp;

        // compute yaw from compensated theta/phi and megnetometer data
        let psi = libm::atan2(mag_y_compensated, mag_x_compensated);

        if !usb_dev.poll(&mut [&mut serial]) {
            continue
        }

        if n % 20 == 0 {
            let mut temp_str: String<512> = String::new();

            temp_str.truncate(0);
            write!(temp_str, "theta: {:?}, phi: {:?}, psi: {:?}\r\n", theta * 57.295779513, phi * 57.295779513, psi * 57.295779513).unwrap();
            write_serial(&mut serial, temp_str.as_str());
            usb_dev.poll(&mut [&mut serial]);

            // temp_str.truncate(0);
            // write!(temp_str, "gyro: {:+10.6}, {:+10.6}, {:+10.6}\r\n", gyro_x, gyro_y, gyro_z).unwrap();
            // write_serial(&mut serial, temp_str.as_str());
            // usb_dev.poll(&mut [&mut serial]);
            //
            // temp_str.truncate(0);
            // write!(temp_str, "acc: {:+10.6}, {:+10.6}, {:+10.6}\r\n", acc_x, acc_y, acc_z).unwrap();
            // write_serial(&mut serial, temp_str.as_str());
            // usb_dev.poll(&mut [&mut serial]);
            //
            // temp_str.truncate(0);
            // write!(temp_str, "mag: {:+10.2}, {:+10.2}, {:+10.2}\r\n", mag_x, mag_y, mag_z).unwrap();
            // write_serial(&mut serial, temp_str.as_str());
            // usb_dev.poll(&mut [&mut serial]);

        }

        let _ = nb::block!(count_down.wait());
    }
}

// End of file
