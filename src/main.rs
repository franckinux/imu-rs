#![no_main]
#![no_std]

mod types;
#[cfg(feature="bmi160")]
mod bmi160;
#[cfg(feature="lsm6ds3")]
mod lsm6ds3;
#[cfg(feature="qmc5883l")]
mod qmc5883l;

// use panic_halt  as _;
use panic_semihosting as _;
use core::fmt::Write;
use cortex_m_rt::entry;
use num_traits::Float;
use crate::hal::pac;
use stm32f4xx_hal as hal;
use hal::{
    i2c::{DutyCycle, I2c, Mode as I2cMode},
    pac::{I2C1, USART1},
    prelude::*,
    serial::{Config, Serial, Tx},
    timer::Counter,
};
use fugit::ExtU32;
use shared_bus::{BusManagerSimple, I2cProxy, NullMutex};
use eeprom24x::{
    addr_size::TwoBytes,
    Eeprom24x,
    Eeprom24xTrait,
    page_size::B64,
    SlaveAddr as EepromSlaveAddr
};
use crate::types::{Accelerometer, Gyroscope, Magnetometer};
#[cfg(feature="bmi160")]
use bmi160::Bmi160Imu;
#[cfg(feature="lsm6ds3")]
use lsm6ds3::Lsm6ds3Imu;
#[cfg(feature="qmc5883l")]
use qmc5883l::Qmc5883lImu;
use crc::{Crc, CRC_16_MODBUS};


#[derive(Debug, Default)]
struct AccelerometerCalibrationData {
    x_offset: f32,
    y_offset: f32,
    z_offset: f32,
    x_scaling_factor: f32,
    y_scaling_factor: f32,
    z_scaling_factor: f32,
    phi_offset: f32,
    theta_offset: f32,
}


#[derive(Debug, Default)]
struct GyroscopeCalibrationData { p_offset: f32, q_offset:f32, r_offset:f32 }
#[derive(Debug, Default)]
struct MagnetometerCalibrationData {
    x_offset: f32,
    y_offset: f32,
    z_offset: f32,
    x_scaling_factor: f32,
    y_scaling_factor: f32,
    z_scaling_factor: f32,
}


fn calibrate_accelerometer(
    sensor: &mut impl Accelerometer,
    tx: &mut Tx<USART1>,
    timer2: &mut Counter<stm32f4xx_hal::pac::TIM2, 1_000_000>,
) -> AccelerometerCalibrationData  {
    let mut x_min = 0.0;
    let mut y_min = 0.0;
    let mut z_min = 0.0;
    let mut x_max = 0.0;
    let mut y_max = 0.0;
    let mut z_max = 0.0;

    let _ = timer2.start(2000.millis());
    nb::block!(timer2.wait()).unwrap();

    let mut phi_offset = 0.0;
    let mut theta_offset = 0.0;
    const ITERATIONS: u16 = 400;  // for 20s

    tx.write_str("---> accelerometer calibration\r\n").unwrap();
    for counter in 0..ITERATIONS {
        let _ = timer2.start(50.millis());

        if counter % 2 == 0 {
            tx.write_char('.').unwrap();
            // tx.flush().unwrap();
        }

        let (x, y, z): (f32, f32, f32) = sensor.acc_read_values().unwrap();

        x_min = f32::min(x, x_min);
        y_min = f32::min(y, y_min);
        z_min = f32::min(z, z_min);
        x_max = f32::max(x, x_max);
        y_max = f32::max(y, y_max);
        z_max = f32::max(z, z_max);

        nb::block!(timer2.wait()).unwrap();
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

    tx.write_char('*').unwrap();

    for counter in 0..ITERATIONS {
        let _ = timer2.start(50.millis());

        if counter % 2 == 0 {
            tx.write_char('.').unwrap();
            // tx.flush().unwrap();
        }

        // get accelerometer data, scaled with sensitivity
        let (mut x, mut y, mut z): (f32, f32, f32) = sensor.acc_read_values().unwrap();
        x = (x - x_offset) * x_scaling_factor;
        y = (y - y_offset) * y_scaling_factor;
        z = (z - z_offset) * z_scaling_factor;

        // compute theta and phi from the accelerometer
        let a = (x * x + y * y + z * z).sqrt();
        let phi = (y / z).atan();
        let theta = (x / a).asin();

        phi_offset += phi;
        theta_offset += theta;

        nb::block!(timer2.wait()).unwrap();
    }

    phi_offset /= ITERATIONS as f32;
    theta_offset /= ITERATIONS as f32;

    writeln!(
        tx,
        "\r\n\nextrema (X, Y, Z): {:?}, {:?}, {:?}, {:?}, {:?}, {:?}\r",
        x_min, x_max, y_min, y_max, z_min, z_max
    ).unwrap();

    writeln!(tx, "offsets (X, Y, Z): {:?}, {:?}, {:?}\r", x_offset, y_offset, z_offset).unwrap();

    writeln!(
        tx,
        "factors (X, Y, Z): {:?}, {:?}, {:?}\r\n\n", x_scaling_factor, y_scaling_factor, z_scaling_factor,
    ).unwrap();

    writeln!(tx, "phi_offset: {:?}, theta_offset: {:?}\r", phi_offset, theta_offset).unwrap();

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
    sensor: &mut impl Gyroscope,
    tx: &mut Tx<USART1>,
    timer2: &mut Counter<stm32f4xx_hal::pac::TIM2, 1_000_000>,
) -> GyroscopeCalibrationData  {
    let _ = timer2.start(2000.millis());
    nb::block!(timer2.wait()).unwrap();

    let mut p_offset = 0.0;
    let mut q_offset = 0.0;
    let mut r_offset = 0.0;
    const ITERATIONS: u16 = 400;  // for 20s

    tx.write_str("\r\n\n---> gyroscope calibration\r\n").unwrap();
    for counter in 0..ITERATIONS {  // 20s
        let _ = timer2.start(50.millis());

        if counter % 2 == 0 {
            tx.write_char('.').unwrap();
            // tx.flush().unwrap();
        }

        // get gyro data, scaled with sensitivity
        let (p, q, r): (f32, f32, f32) = sensor.gyro_read_values().unwrap();

        p_offset += p;
        q_offset += q;
        r_offset += r;

        nb::block!(timer2.wait()).unwrap();
    }

    p_offset /= ITERATIONS as f32;
    q_offset /= ITERATIONS as f32;
    r_offset /= ITERATIONS as f32;

    writeln!(
        tx,
        "\r\n\np_offset: {:?}, q_offset: {:?}, r_offset: {:?}\r", p_offset, q_offset, r_offset
    ).unwrap();

    GyroscopeCalibrationData { p_offset, q_offset, r_offset }
}


fn calibrate_magnetometer(
    sensor: &mut impl Magnetometer,
    tx: &mut Tx<USART1>,
    timer2: &mut Counter<stm32f4xx_hal::pac::TIM2, 1_000_000>,
) -> MagnetometerCalibrationData {
    let mut x_min = 0.0;
    let mut y_min = 0.0;
    let mut z_min = 0.0;
    let mut x_max = 0.0;
    let mut y_max = 0.0;
    let mut z_max = 0.0;
    const ITERATIONS: u16 = 400;

    let _ = timer2.start(2000.millis());
    nb::block!(timer2.wait()).unwrap();

    tx.write_str("\r\n\n---> magnetometer calibration\r\n").unwrap();
    for counter in 0..ITERATIONS {  // 20s
        let _ = timer2.start(50.millis());

        if counter % 2 == 0 {
            tx.write_char('.').unwrap();
            // tx.flush().unwrap();
        }

        let (x, y, z) = sensor.mag_read_values().unwrap();

        x_min = f32::min(x, x_min);
        y_min = f32::min(y, y_min);
        z_min = f32::min(z, z_min);
        x_max = f32::max(x, x_max);
        y_max = f32::max(y, y_max);
        z_max = f32::max(z, z_max);

        nb::block!(timer2.wait()).unwrap();
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

    writeln!(
        tx,
        "\r\n\nextrema (X, Y, Z): {:?}, {:?}, {:?}, {:?}, {:?}, {:?}\r",
        x_min, x_max, y_min, y_max, z_min, z_max
    ).unwrap();

    writeln!(tx, "offsets (X, Y, Z): {:?}, {:?}, {:?}\r", x_offset, y_offset, z_offset).unwrap();

    writeln!(
        tx,
        "factors (X, Y, Z): {:?}, {:?}, {:?}\r\n", x_scaling_factor, y_scaling_factor, z_scaling_factor,
    ).unwrap();

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


fn save_calibration_data(
    delay: &mut cortex_m::delay::Delay,
    eeprom: &mut Eeprom24x<I2cProxy<'_, NullMutex<stm32f4xx_hal::i2c::I2c<I2C1>>>, B64, TwoBytes, eeprom24x::unique_serial::No>,
    acc: &AccelerometerCalibrationData, gyro: &GyroscopeCalibrationData, mag: &MagnetometerCalibrationData
) {
    let mut buffer = [0u8; 70];

    buffer[0..4].clone_from_slice(&acc.x_offset.to_be_bytes());
    buffer[4..8].clone_from_slice(&acc.y_offset.to_be_bytes());
    buffer[8..12].clone_from_slice(&acc.z_offset.to_be_bytes());
    buffer[12..16].clone_from_slice(&acc.x_scaling_factor.to_be_bytes());
    buffer[16..20].clone_from_slice(&acc.y_scaling_factor.to_be_bytes());
    buffer[20..24].clone_from_slice(&acc.z_scaling_factor.to_be_bytes());
    buffer[24..28].clone_from_slice(&acc.phi_offset.to_be_bytes());
    buffer[28..32].clone_from_slice(&acc.theta_offset.to_be_bytes());
    buffer[32..36].clone_from_slice(&gyro.p_offset.to_be_bytes());
    buffer[36..40].clone_from_slice(&gyro.q_offset.to_be_bytes());
    buffer[40..44].clone_from_slice(&gyro.r_offset.to_be_bytes());
    buffer[44..48].clone_from_slice(&mag.x_offset.to_be_bytes());
    buffer[48..52].clone_from_slice(&mag.y_offset.to_be_bytes());
    buffer[52..56].clone_from_slice(&mag.z_offset.to_be_bytes());
    buffer[56..60].clone_from_slice(&mag.x_scaling_factor.to_be_bytes());
    buffer[60..64].clone_from_slice(&mag.y_scaling_factor.to_be_bytes());
    buffer[64..68].clone_from_slice(&mag.z_scaling_factor.to_be_bytes());

    // compute the crc of the 68 first bytes as a u16
    let crc = crc_modbus(&buffer[0..68]);
    buffer[68..70].clone_from_slice(&crc.to_be_bytes());

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
    eeprom: &mut Eeprom24x<I2cProxy<'_, NullMutex<stm32f4xx_hal::i2c::I2c<I2C1>>>, B64, TwoBytes, eeprom24x::unique_serial::No>,
) -> Option<
    (AccelerometerCalibrationData, GyroscopeCalibrationData, MagnetometerCalibrationData)
> {
    let mut buffer = [0u8; 70];
    eeprom.read_data(0, &mut buffer[..]).unwrap();

    // compute the crc of the 68 first bytes as a u16
    let crc = crc_modbus(&buffer[0..68]);
    let crc2 = u16::from_be_bytes(buffer[68..70].try_into().unwrap());

    if crc == crc2 {
        // create default structs
        let mut acc = AccelerometerCalibrationData::default();
        let mut gyro = GyroscopeCalibrationData::default();
        let mut mag = MagnetometerCalibrationData::default();

        // fill the fields with the values in the array
        acc.x_offset = f32::from_be_bytes(buffer[0..4].try_into().unwrap());
        acc.y_offset = f32::from_be_bytes(buffer[4..8].try_into().unwrap());
        acc.z_offset = f32::from_be_bytes(buffer[8..12].try_into().unwrap());
        acc.x_scaling_factor = f32::from_be_bytes(buffer[12..16].try_into().unwrap());
        acc.y_scaling_factor = f32::from_be_bytes(buffer[16..20].try_into().unwrap());
        acc.z_scaling_factor = f32::from_be_bytes(buffer[20..24].try_into().unwrap());
        acc.phi_offset = f32::from_be_bytes(buffer[24..28].try_into().unwrap());
        acc.theta_offset = f32::from_be_bytes(buffer[28..32].try_into().unwrap());
        gyro.p_offset = f32::from_be_bytes(buffer[32..36].try_into().unwrap());
        gyro.q_offset = f32::from_be_bytes(buffer[36..40].try_into().unwrap());
        gyro.r_offset = f32::from_be_bytes(buffer[40..44].try_into().unwrap());
        mag.x_offset = f32::from_be_bytes(buffer[44..48].try_into().unwrap());
        mag.y_offset = f32::from_be_bytes(buffer[48..52].try_into().unwrap());
        mag.z_offset = f32::from_be_bytes(buffer[52..56].try_into().unwrap());
        mag.x_scaling_factor = f32::from_be_bytes(buffer[56..60].try_into().unwrap());
        mag.y_scaling_factor = f32::from_be_bytes(buffer[60..64].try_into().unwrap());
        mag.z_scaling_factor = f32::from_be_bytes(buffer[64..68].try_into().unwrap());

        Some((acc, gyro, mag))
    } else {
        None
    }
}


#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    // let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(48.MHz()).hclk(48.MHz()).freeze();
    let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(80.MHz()).hclk(80.MHz()).freeze();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // various inputs / outputs
    let mut led = gpioc.pc13.into_push_pull_output();
    let setup_pin = gpioa.pa0.into_pull_up_input();

    // Configure serial link
    let tx = gpioa.pa9.into_alternate::<7>();
    let rx = gpioa.pa10.into_alternate::<7>();

    let serial1: Serial<stm32f4xx_hal::pac::USART1, u8> = Serial::new(
        dp.USART1,
        (tx, rx),
        Config::default().baudrate(115200.bps()),
        &clocks
    ).unwrap();
    let mut tx = serial1.split().0;

    // Configre I2C bus
    let scl = gpiob.pb6.into_alternate_open_drain();
    let sda = gpiob.pb7.into_alternate_open_drain();

    // let i2c_mode = I2cMode::Standard{frequency: 100.kHz()};
    let i2c_mode = I2cMode::Fast{frequency: 400.kHz(), duty_cycle: DutyCycle::Ratio2to1};
    let i2c_bus = I2c::new(dp.I2C1, (scl, sda), i2c_mode, &clocks);
    let i2c_bus = BusManagerSimple::new(i2c_bus);

    // === AT24C256 ===
    let mut eeprom = Eeprom24x::new_24x256(i2c_bus.acquire_i2c(), EepromSlaveAddr::default());

    // === Accelerometer / gyroscope ===
    #[cfg(feature="bmi160")]
    let mut acc_gyro = Bmi160Imu::new(i2c_bus.acquire_i2c()).unwrap();
    #[cfg(feature="lsm6ds3")]
    let mut acc_gyro = Lsm6ds3Imu::new(i2c_bus.acquire_i2c()).unwrap();
    acc_gyro.acc_configure().unwrap();
    acc_gyro.gyro_configure().unwrap();

    // === Magnetometer ===
    #[cfg(feature="qmc5883l")]
    let mut mag = Qmc5883lImu::new(i2c_bus.acquire_i2c()).unwrap();
    mag.mag_configure().unwrap();

    // The delay object lets us wait for specified amounts of time (in milliseconds)
    let mut delay = cortex_m::delay::Delay::new(cp.SYST, 80_000_000);
    let mut timer2 = dp.TIM2.counter_us(&clocks);

    // calibrations
    let acc_calibration_data;
    let gyro_calibration_data;
    let mag_calibration_data;
    if setup_pin.is_low() {
        acc_calibration_data = calibrate_accelerometer(&mut acc_gyro, &mut tx, &mut timer2);
        gyro_calibration_data = calibrate_gyroscope(&mut acc_gyro, &mut tx, &mut timer2);
        mag_calibration_data = calibrate_magnetometer(&mut mag, &mut tx, &mut timer2);
        save_calibration_data(
            &mut delay,
            &mut eeprom,
            &acc_calibration_data,
            &gyro_calibration_data,
            &mag_calibration_data
        );
    } else {
        // cannot use "let if" construction here because there would have no action on
        // the positive arm and the compiler would complain about uninitialized struct
        let calibration_data = restore_calibration_data(&mut eeprom);
        if calibration_data.is_some() {
            tx.write_str("OK_CRC").unwrap();
            (acc_calibration_data, gyro_calibration_data, mag_calibration_data) = calibration_data.unwrap();
        } else {
            tx.write_str("ERR_CRC").unwrap();
            acc_calibration_data = Default::default();
            gyro_calibration_data = Default::default();
            mag_calibration_data = MagnetometerCalibrationData {
                x_scaling_factor: 1.0,
                y_scaling_factor: 1.0,
                z_scaling_factor: 1.0,
                ..Default::default()
            }
        }
    }

    // main loop
    const PERIOD: u32 = 4000;  // for 250 Hz
    let mut count = 0;
    let period = PERIOD.micros();
    let _ = timer2.start(period);
    let mut buffer: [u8;  38] = [0u8; 38];
    buffer[0] = 0x55;
    buffer[1] = 0xaa;

    loop {
        nb::block!(timer2.wait()).unwrap();
        let _ = timer2.start(period);

        // get raw values from sensors
        let result = acc_gyro.acc_read_values();
        if result.is_err() {
            continue;
        };
        let (mut acc_x, mut acc_y, mut acc_z): (f32, f32, f32) = result.unwrap();

        let result = acc_gyro.gyro_read_values();
        if result.is_err() {
            continue;
        };
        let (mut gyro_p, mut gyro_q, mut gyro_r): (f32, f32, f32) = result.unwrap();

        let result = mag.mag_read_values();
        if result.is_err() {
            continue;
        };
        let (mut mag_x, mut mag_y, mut mag_z): (f32, f32, f32) = result.unwrap();

        // print raw values for debug
        // writeln!(tx, "acc: (x: {:0.6}, y: {:0.6}, z: {:0.6})\r", acc_x, acc_y, acc_z).unwrap();
        // writeln!(tx, "gyro: (x: {:0.6}, y: {:0.6}, z: {:0.6})\r", gyro_p, gyro_q, gyro_r).unwrap();
        // writeln!(tx, "mag: (x: {:0.6}, y: {:0.6}, z: {:0.6})\r", mag_x, mag_y, mag_z).unwrap();

        // apply calibration correction
        acc_x = (acc_x - acc_calibration_data.x_offset) * acc_calibration_data.x_scaling_factor;
        acc_y = (acc_y - acc_calibration_data.y_offset) * acc_calibration_data.y_scaling_factor;
        acc_z = (acc_z - acc_calibration_data.z_offset) * acc_calibration_data.z_scaling_factor;

        gyro_p -= gyro_calibration_data.p_offset;
        gyro_q -= gyro_calibration_data.q_offset;
        gyro_r -= gyro_calibration_data.r_offset;

        mag_x = (mag_x - mag_calibration_data.x_offset) * mag_calibration_data.x_scaling_factor;
        mag_y = (mag_y - mag_calibration_data.y_offset) * mag_calibration_data.y_scaling_factor;
        mag_z = (mag_z - mag_calibration_data.z_offset) * mag_calibration_data.z_scaling_factor;

        // print values as f32 bytes
        buffer[2..6].clone_from_slice(&(acc_x as f32).to_be_bytes());
        buffer[6..10].clone_from_slice(&(acc_y as f32).to_be_bytes());
        buffer[10..14].clone_from_slice(&(acc_z as f32).to_be_bytes());
        buffer[14..18].clone_from_slice(&(gyro_p as f32).to_be_bytes());
        buffer[18..22].clone_from_slice(&(gyro_q as f32).to_be_bytes());
        buffer[22..26].clone_from_slice(&(gyro_r as f32).to_be_bytes());
        buffer[26..30].clone_from_slice(&(mag_x as f32).to_be_bytes());
        buffer[30..34].clone_from_slice(&(mag_y as f32).to_be_bytes());
        buffer[34..38].clone_from_slice(&(mag_z as f32).to_be_bytes());
        tx.bwrite_all(&buffer).unwrap();

        // 1s period 1s
        if count == 125 {
            led.toggle();
            count = 0;
        } else {
            count += 1;
        }
    }
}
