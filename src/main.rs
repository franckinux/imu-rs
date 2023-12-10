#![no_std]
#![no_main]

// Used frame : ENU
// Rotation order is ZYX

// psi (Ψψ) = yaw = lacet
// theta (Θθ) = pitch = tangage
// phi a(Φφ) = roll = roulis

use core::{
    fmt::{Debug, Write},
    iter::once,
};

use embedded_hal::{
    digital::v2::InputPin,
    timer::CountDown
};
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
    gpio::{bank0::Gpio14, bank0::Gpio15, FunctionI2C, FunctionI2c, Pin, PullUp},
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

use crc::{Crc, CRC_16_MODBUS};

use shared_bus::{
    BusManagerSimple,
    I2cProxy, NullMutex,
};
use eeprom24x::{
    addr_size::TwoBytes,
    Eeprom24x,
    Eeprom24xTrait,
    page_size::B64,
    SlaveAddr
};
use mpu6050::Mpu6050;
use qmc5883l::QMC5883L;

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

#[derive(Debug, Default)]
struct AccelerometerCalibrationData {
    x_offset: f64,
    y_offset: f64,
    z_offset: f64,
    x_scaling_factor: f64,
    y_scaling_factor: f64,
    z_scaling_factor: f64,
    phi_offset: f64,
    theta_offset:f64,
}
#[derive(Debug, Default)]
struct GyroscopeCalibrationData { p_offset: f64, q_offset:f64, r_offset:f64 }
#[derive(Debug, Default)]
struct MagnetometerCalibrationData {
    x_offset: f64,
    y_offset: f64,
    z_offset: f64,
    x_scaling_factor: f64,
    y_scaling_factor: f64,
    z_scaling_factor: f64,
}


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


fn write_serial(serial: &mut SerialPort<hal::usb::UsbBus>, string: &str)  {
    match serial.write(string.as_bytes()) {
        Ok(_count) => {
            // count bytes were written
        },
        Err(UsbError::WouldBlock) => {},  // No data could be written (buffers full)
        Err(_err) => {},  // An error occurred
    };
}


fn accelerometer_mpu6050_convert_and_swap(x: f32, y: f32, z: f32) -> (f64, f64, f64) {
    (-x as f64, -y as f64, -z as f64)
}


fn gyroscope_mpu6050_convert_and_swap(p: f32, q: f32, r: f32) -> (f64, f64, f64) {
    (p as f64, q as f64, r as f64)
}


fn magnetometer_qmc5883l_convert_and_swap(x: i16, y: i16, z: i16) -> (f64, f64, f64) {
    (-x as f64, -y as f64, -z as f64)
}


fn calibrate_accelerometer(
    mpu: &mut Mpu6050<I2cProxy<'_, NullMutex<hal::I2C<pac::I2C1, (hal::gpio::Pin<Gpio14, FunctionI2c, PullUp>, hal::gpio::Pin<Gpio15, FunctionI2c, PullUp>)>>>>,
    serial: &mut SerialPort<hal::usb::UsbBus>,
    timer: &hal::Timer,
    usb_dev: &mut UsbDevice<'_, hal::usb::UsbBus>,
) -> AccelerometerCalibrationData  {
    let mut x_min = 0.0;
    let mut y_min = 0.0;
    let mut z_min = 0.0;
    let mut x_max = 0.0;
    let mut y_max = 0.0;
    let mut z_max = 0.0;

    let mut count_down = timer.count_down();

    for _ in 0..200 {  // 2s
        count_down.start(10.millis());
        usb_dev.poll(&mut [serial]);
        let _ = nb::block!(count_down.wait());
    }

    let mut phi_offset = 0.0;
    let mut theta_offset = 0.0;
    const ITERATIONS: u16 = 400;

    write_serial(serial, "---> accelerometer calibration\r\n\n");
    for counter in 0..ITERATIONS {  // 20s
        count_down.start(50.millis());

        if counter % 2 == 0 {
            write_serial(serial, ".");
            serial.flush().unwrap();
        }

        let acc = mpu.get_acc().unwrap();
        let (x, y, z) = accelerometer_mpu6050_convert_and_swap(acc.x, acc.y, acc.z);

        x_min = f64::min(x, x_min);
        y_min = f64::min(y, y_min);
        z_min = f64::min(z, z_min);
        x_max = f64::max(x, x_max);
        y_max = f64::max(y, y_max);
        z_max = f64::max(z, z_max);

        usb_dev.poll(&mut [serial]);
        let _ = nb::block!(count_down.wait());
    }

    let x_offset = (x_min + x_max) / 2.0;
    let y_offset = (y_min + y_max) / 2.0;
    let z_offset = (z_min + z_max) / 2.0;

    let x_chord = (x_max - x_min) / 2.0;
    let y_chord = (y_max - y_min) / 2.0;
    let z_chord = (z_max - z_min) / 2.0;

    let mean_chord = (x_chord + y_chord + z_chord) / 3.0;

    let x_scaling_factor = mean_chord / x_chord;
    let y_scaling_factor = mean_chord / y_chord;
    let z_scaling_factor = mean_chord / z_chord;

    write_serial(serial, "\r\n");

    for counter in 0..ITERATIONS {  // 20s
        count_down.start(50.millis());

        if counter % 2 == 0 {
            write_serial(serial, ".");
            serial.flush().unwrap();
        }

        // get accelerometer data, scaled with sensitivity
        let acc = mpu.get_acc().unwrap();
        let (mut x, mut y, mut z) = accelerometer_mpu6050_convert_and_swap(acc.x, acc.y, acc.z);
        x = (x - x_offset) * x_scaling_factor;
        y = (y - y_offset) * y_scaling_factor;
        z = (z - z_offset) * z_scaling_factor;

        // compute theta and phi from the accelerometer
        let a = libm::sqrt(x * x + y * y + z * z);
        let phi = -libm::asin(y / a);
        let theta = libm::asin(x / a);

        phi_offset += phi;
        theta_offset += theta;

        usb_dev.poll(&mut [serial]);
        let _ = nb::block!(count_down.wait());
    }

    phi_offset /= ITERATIONS as f64;
    theta_offset /= ITERATIONS as f64;

    let mut temp_str: String<256> = String::new();

    write!(
        temp_str,
        "\r\n\nextrema (X, Y, Z): {:?}, {:?}, {:?}, {:?}, {:?}, {:?}\r\n",
        x_min, x_max, y_min, y_max, z_min, z_max
    ).unwrap();
    write_serial(serial, temp_str.as_str());

    temp_str.truncate(0);
    write!(temp_str, "offsets (X, Y, Z): {:?}, {:?}, {:?}\r\n", x_offset, y_offset, z_offset).unwrap();
    write_serial(serial, temp_str.as_str());

    temp_str.truncate(0);
    write!(
        temp_str,
        "factors (X, Y, Z): {:?}, {:?}, {:?}\r\n\n", x_scaling_factor, y_scaling_factor, z_scaling_factor,
    ).unwrap();
    write_serial(serial, temp_str.as_str());
    usb_dev.poll(&mut [serial]);

    temp_str.truncate(0);
    write!(
        temp_str,
        "\r\n\n: phi_offset: {:?}, theta_offset: {:?}\r\n", phi_offset, theta_offset
    ).unwrap();
    write_serial(serial, temp_str.as_str());
    usb_dev.poll(&mut [serial]);

    AccelerometerCalibrationData {
        x_offset,
        y_offset,
        z_offset,
        x_scaling_factor,
        y_scaling_factor,
        z_scaling_factor,
        phi_offset,
        theta_offset
    }
}


fn calibrate_gyroscope(
    mpu: &mut Mpu6050<I2cProxy<'_, NullMutex<hal::I2C<pac::I2C1, (hal::gpio::Pin<Gpio14, FunctionI2c, PullUp>, hal::gpio::Pin<Gpio15, FunctionI2c, PullUp>)>>>>,
    serial: &mut SerialPort<hal::usb::UsbBus>,
    timer: &hal::Timer,
    usb_dev: &mut UsbDevice<'_, hal::usb::UsbBus>,
) -> GyroscopeCalibrationData  {
    let mut count_down = timer.count_down();

    for _ in 0..200 {  // 2s
        count_down.start(10.millis());
        usb_dev.poll(&mut [serial]);
        let _ = nb::block!(count_down.wait());
    }

    let mut p_offset = 0.0;
    let mut q_offset = 0.0;
    let mut r_offset = 0.0;
    const ITERATIONS: u16 = 400;

    write_serial(serial, "---> gyroscope calibration\r\n\n");
    for counter in 0..ITERATIONS {  // 20s
        count_down.start(50.millis());

        if counter % 2 == 0 {
            write_serial(serial, ".");
            serial.flush().unwrap();
        }

        // get gyro data, scaled with sensitivity
        let gyro = mpu.get_gyro().unwrap();
        let (p, q, r) = gyroscope_mpu6050_convert_and_swap(gyro.x, gyro.y, gyro.z);

        p_offset += p;
        q_offset += q;
        r_offset += r;

        usb_dev.poll(&mut [serial]);
        let _ = nb::block!(count_down.wait());
    }

    p_offset /= ITERATIONS as f64;
    q_offset /= ITERATIONS as f64;
    r_offset /= ITERATIONS as f64;

    let mut temp_str: String<256> = String::new();

    write!(
        temp_str,
        "\r\n\n: p_offset: {:?}, q_offset: {:?}, r_offset: {:?}\r\n", p_offset, q_offset, r_offset
    ).unwrap();
    write_serial(serial, temp_str.as_str());
    usb_dev.poll(&mut [serial]);

    GyroscopeCalibrationData { p_offset, q_offset, r_offset }
}


fn calibrate_magnetometer(
    qmc: &mut QMC5883L<I2cProxy<'_, NullMutex<hal::I2C<pac::I2C1, (hal::gpio::Pin<Gpio14, FunctionI2c, PullUp>, hal::gpio::Pin<Gpio15, FunctionI2c, PullUp>)>>>>,
    serial: &mut SerialPort<hal::usb::UsbBus>,
    timer: &hal::Timer,
    usb_dev: &mut UsbDevice<'_, hal::usb::UsbBus>,
) -> MagnetometerCalibrationData {
    let mut x_min = 0.0;
    let mut y_min = 0.0;
    let mut z_min = 0.0;
    let mut x_max = 0.0;
    let mut y_max = 0.0;
    let mut z_max = 0.0;
    const ITERATIONS: u16 = 400;

    let mut count_down = timer.count_down();

    for _ in 0..500 {  // 5s
        count_down.start(10.millis());
        usb_dev.poll(&mut [serial]);
        let _ = nb::block!(count_down.wait());
    }

    write_serial(serial, "---> magnetometer calibration\r\n\n");
    for counter in 0..ITERATIONS {  // 20s
        count_down.start(50.millis());

        if counter % 2 == 0 {
            write_serial(serial, ".");
            serial.flush().unwrap();
        }

        let (x, y, z) = qmc.mag().unwrap();
        let (x, y, z) = magnetometer_qmc5883l_convert_and_swap(x, y, z);

        x_min = f64::min(x, x_min);
        y_min = f64::min(y, y_min);
        z_min = f64::min(z, z_min);
        x_max = f64::max(x, x_max);
        y_max = f64::max(y, y_max);
        z_max = f64::max(z, z_max);

        usb_dev.poll(&mut [serial]);
        let _ = nb::block!(count_down.wait());
    }

    let x_offset = (x_min + x_max) / 2.0;
    let y_offset = (y_min + y_max) / 2.0;
    let z_offset = (z_min + z_max) / 2.0;

    let x_chord = (x_max - x_min) / 2.0;
    let y_chord = (y_max - y_min) / 2.0;
    let z_chord = (z_max - z_min) / 2.0;

    let mean_chord = (x_chord + y_chord + z_chord) / 3.0;

    let x_scaling_factor = mean_chord / x_chord;
    let y_scaling_factor = mean_chord / y_chord;
    let z_scaling_factor = mean_chord / z_chord;

    let mut temp_str: String<256> = String::new();

    write!(
        temp_str,
        "\r\n\nextrema (X, Y, Z): {:?}, {:?}, {:?}, {:?}, {:?}, {:?}\r\n",
        x_min, x_max, y_min, y_max, z_min, z_max
    ).unwrap();
    write_serial(serial, temp_str.as_str());

    temp_str.truncate(0);
    write!(temp_str, "offsets (X, Y, Z): {:?}, {:?}, {:?}\r\n", x_offset, y_offset, z_offset).unwrap();
    write_serial(serial, temp_str.as_str());

    temp_str.truncate(0);
    write!(
        temp_str,
        "factors (X, Y, Z): {:?}, {:?}, {:?}\r\n\n", x_scaling_factor, y_scaling_factor, z_scaling_factor,
    ).unwrap();
    write_serial(serial, temp_str.as_str());
    usb_dev.poll(&mut [serial]);

    MagnetometerCalibrationData {
        x_offset, y_offset, z_offset, x_scaling_factor, y_scaling_factor, z_scaling_factor
    }
}


fn crc_modbus(buffer: &[u8]) -> u16 {
    let crc = Crc::<u16>::new(&CRC_16_MODBUS);
	let mut digest = crc.digest();
	digest.update(buffer);
    digest.finalize()
}


fn save_calibration_data<E: Eeprom24xTrait>(
    delay: &mut cortex_m::delay::Delay,
    eeprom: &mut E,
    acc: &AccelerometerCalibrationData, gyro: &GyroscopeCalibrationData, mag: &MagnetometerCalibrationData
) where <E as Eeprom24xTrait>::Error: Debug {
    let mut buffer = [0u8; 138];

    buffer[0..8].clone_from_slice(&acc.x_offset.to_be_bytes());
    buffer[8..16].clone_from_slice(&acc.y_offset.to_be_bytes());
    buffer[16..24].clone_from_slice(&acc.z_offset.to_be_bytes());
    buffer[24..32].clone_from_slice(&acc.x_scaling_factor.to_be_bytes());
    buffer[32..40].clone_from_slice(&acc.y_scaling_factor.to_be_bytes());
    buffer[40..48].clone_from_slice(&acc.z_scaling_factor.to_be_bytes());
    buffer[48..56].clone_from_slice(&acc.phi_offset.to_be_bytes());
    buffer[56..64].clone_from_slice(&acc.theta_offset.to_be_bytes());
    buffer[64..72].clone_from_slice(&gyro.p_offset.to_be_bytes());
    buffer[72..80].clone_from_slice(&gyro.q_offset.to_be_bytes());
    buffer[80..88].clone_from_slice(&gyro.r_offset.to_be_bytes());
    buffer[88..96].clone_from_slice(&mag.x_offset.to_be_bytes());
    buffer[96..104].clone_from_slice(&mag.y_offset.to_be_bytes());
    buffer[104..112].clone_from_slice(&mag.z_offset.to_be_bytes());
    buffer[112..120].clone_from_slice(&mag.x_scaling_factor.to_be_bytes());
    buffer[120..128].clone_from_slice(&mag.y_scaling_factor.to_be_bytes());
    buffer[128..136].clone_from_slice(&mag.z_scaling_factor.to_be_bytes());

    // compute the crc of the 136 first bytes as a u16
    let crc = crc_modbus(&buffer[0..136]);
    buffer[136..138].clone_from_slice(&crc.to_be_bytes());

    // write pages
    let page_size = eeprom.page_size();
    let mut start_address = 0;
    for _ in 0..buffer.len() / page_size {
        eeprom.write_page(start_address as u32, &buffer[start_address..start_address + page_size]).unwrap();
        start_address += page_size;
        delay.delay_ms(5);
    }
    // write last page
    if start_address < buffer.len() {
        eeprom.write_page(start_address as u32, &buffer[start_address..]).unwrap();
    }
}


fn restore_calibration_data(
    eeprom: &mut Eeprom24x<I2cProxy<'_, NullMutex<hal::I2C<pac::I2C1, (hal::gpio::Pin<Gpio14, FunctionI2c, PullUp>, hal::gpio::Pin<Gpio15, FunctionI2c, PullUp>)>>>, B64, TwoBytes>
) -> Option<
    (AccelerometerCalibrationData, GyroscopeCalibrationData, MagnetometerCalibrationData)
> {
    let mut buffer = [0u8; 138];
    eeprom.read_data(0, &mut buffer[..]).unwrap();

    // compute the crc of the 136 first bytes as a u16
    let crc = crc_modbus(&buffer[0..136]);
    let crc2 = u16::from_be_bytes(buffer[136..138].try_into().unwrap());

    if crc == crc2 {
        // create default structs
        let mut acc = AccelerometerCalibrationData::default();
        let mut gyro = GyroscopeCalibrationData::default();
        let mut mag = MagnetometerCalibrationData::default();

        // fill the fields with the values in the array
        acc.x_offset= f64::from_be_bytes(buffer[0..8].try_into().unwrap());
        acc.y_offset= f64::from_be_bytes(buffer[8..16].try_into().unwrap());
        acc.z_offset= f64::from_be_bytes(buffer[16..24].try_into().unwrap());
        acc.x_scaling_factor= f64::from_be_bytes(buffer[24..32].try_into().unwrap());
        acc.y_scaling_factor= f64::from_be_bytes(buffer[32..40].try_into().unwrap());
        acc.z_scaling_factor= f64::from_be_bytes(buffer[40..48].try_into().unwrap());
        acc.phi_offset = f64::from_be_bytes(buffer[48..56].try_into().unwrap());
        acc.theta_offset = f64::from_be_bytes(buffer[56..64].try_into().unwrap());
        gyro.p_offset = f64::from_be_bytes(buffer[64..72].try_into().unwrap());
        gyro.q_offset = f64::from_be_bytes(buffer[72..80].try_into().unwrap());
        gyro.r_offset = f64::from_be_bytes(buffer[80..88].try_into().unwrap());
        mag.x_offset= f64::from_be_bytes(buffer[88..96].try_into().unwrap());
        mag.y_offset= f64::from_be_bytes(buffer[96..104].try_into().unwrap());
        mag.z_offset= f64::from_be_bytes(buffer[104..112].try_into().unwrap());
        mag.x_scaling_factor= f64::from_be_bytes(buffer[112..120].try_into().unwrap());
        mag.y_scaling_factor= f64::from_be_bytes(buffer[120..128].try_into().unwrap());
        mag.z_scaling_factor= f64::from_be_bytes(buffer[128..136].try_into().unwrap());

        Some((acc, gyro, mag))
    } else {
        None
    }
}


/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, then performs some example
/// SPI transactions, then goes to sleep.
#[hal::entry]
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
    ) .ok().unwrap();

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

    // pins
    // input is pulled down by a switch to indicat that calibration is requested
    let calibration_pin = pins.gpio8.into_pull_up_input();
    calibration_pin.set_schmitt_enabled(true);

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
        .device_class(USB_CLASS_CDC)  // from: https://www.usb.org/defined-class-codes
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
    let mut eeprom = Eeprom24x::new_24x256(i2c_bus.acquire_i2c(), SlaveAddr::default());

    // === MPU6050 ===
    let mut mpu = Mpu6050::new(i2c_bus.acquire_i2c());
    mpu.init(&mut delay).unwrap();

    // === QMC5883 ===
    let mut qmc = QMC5883L::new(i2c_bus.acquire_i2c()).unwrap();
    qmc.continuous().unwrap();

    let mut count_down = timer.count_down();

    // calibrations
    let acc_calibration_data;
    let _gyro_calibration_data;
    let mag_calibration_data;
    if calibration_pin.is_high().unwrap() {
        acc_calibration_data = calibrate_accelerometer(&mut mpu, &mut serial, &timer, &mut usb_dev);
        _gyro_calibration_data = calibrate_gyroscope(&mut mpu, &mut serial, &timer, &mut usb_dev);
        mag_calibration_data = calibrate_magnetometer(&mut qmc, &mut serial, &timer, &mut usb_dev);
        save_calibration_data(
            &mut delay,
            &mut eeprom,
            &acc_calibration_data,
            &_gyro_calibration_data,
            &mag_calibration_data
        );
    } else {
        // cannot use "let if" construction here because there would have no action on
        // the positive arm and the compiler would complain about uninitialized struct
        let calibration_data = restore_calibration_data(&mut eeprom);
        if calibration_data.is_some() {
            (acc_calibration_data, _gyro_calibration_data, mag_calibration_data) = calibration_data.unwrap();
        } else {
            acc_calibration_data = Default::default();
            _gyro_calibration_data = Default::default();
            mag_calibration_data = MagnetometerCalibrationData {
                x_scaling_factor: 1.0,
                y_scaling_factor: 1.0,
                z_scaling_factor: 1.0,
                ..Default::default()
            }
        }
    }

    // Infinite colour wheel loop
    let mut n: u8 = 128;
    loop {
        count_down.start(50.millis());

        ws.write(brightness(once(wheel(n)), 32)).unwrap();
        n = n.wrapping_add(1);

        // get accelerometer data, scaled with sensitivity
        let acc = mpu.get_acc().unwrap();
        let (mut acc_x, mut acc_y, mut acc_z) = accelerometer_mpu6050_convert_and_swap(acc.x, acc.y, acc.z);
        acc_x = (acc_x - acc_calibration_data.x_offset) * acc_calibration_data.x_scaling_factor;
        acc_y = (acc_y - acc_calibration_data.y_offset) * acc_calibration_data.y_scaling_factor;
        acc_z = (acc_z - acc_calibration_data.z_offset) * acc_calibration_data.z_scaling_factor;

        // get gyro data, scaled with sensitivity
        let gyro = mpu.get_gyro().unwrap();
        let (mut _gyro_p, mut _gyro_q, mut _gyro_r) = gyroscope_mpu6050_convert_and_swap(gyro.x, gyro.y, gyro.z);
        _gyro_p -= _gyro_calibration_data.p_offset;
        _gyro_q -= _gyro_calibration_data.q_offset;
        _gyro_r -= _gyro_calibration_data.r_offset;

        let (mag_x, mag_y, mag_z) = qmc.mag().unwrap();
        let (mut mag_x, mut mag_y, mut mag_z) = magnetometer_qmc5883l_convert_and_swap(mag_x, mag_y, mag_z);
        mag_x = (mag_x - mag_calibration_data.x_offset) * mag_calibration_data.x_scaling_factor;
        mag_y = (mag_y - mag_calibration_data.y_offset) * mag_calibration_data.y_scaling_factor;
        mag_z = (mag_z - mag_calibration_data.z_offset) * mag_calibration_data.z_scaling_factor;

        // compute theta and phi from the accelerometer
        let a = libm::sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
        let phi = -libm::asin(acc_y / a) - acc_calibration_data.phi_offset;
        let theta = libm::asin(acc_x / a) - acc_calibration_data.theta_offset;

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

        if n % 4 == 0 {
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
