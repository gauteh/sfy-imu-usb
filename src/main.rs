#![no_std]
#![no_main]

use defmt_rtt as _;

defmt::timestamp!("{=i32}", {0});

// we use this for defs of sinf etc.
extern crate cmsis_dsp;

use ambiq_hal::{self as hal, prelude::*};
use chrono::NaiveDate;
use core::cell::RefCell;
use core::fmt::{Debug, Write as _};
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicI32, Ordering};
#[allow(unused_imports)]
use cortex_m::{
    asm,
    interrupt::{free, Mutex},
};
use cortex_m_rt::{entry, exception, ExceptionFrame};
use embedded_hal::blocking::{
    delay::DelayMs,
    i2c::{Read, Write, WriteRead},
};

use ism330dhcx::{ctrl1xl, ctrl2g, fifo, fifoctrl, Ism330Dhcx};

use ufmt::{uwrite, uwriteln};
// use ufmt_float::{uFmt_f32, uFmt_f64};
use ufloat::Uf64;

use hal::{i2c, pac::interrupt};

#[derive(Debug)]
pub enum ImuError<E: Debug> {
    I2C(E),
    FifoOverrun {
        fifo_full: bool,
        overrun: bool,
        latched: bool,
        samples: u16,
    },
    FifoBadSequence(fifo::Value, fifo::Value),
    TooFewSamples(i64),
}

impl<E: Debug> From<E> for ImuError<E> {
    fn from(e: E) -> ImuError<E> {
        ImuError::I2C(e)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Freq {
    Hz26,
    Hz52,
    Hz104,
    Hz208,
    Hz833,
}

impl Freq {
    pub const fn value(&self) -> f32 {
        use Freq::*;

        match self {
            Hz26 => 26.,
            Hz52 => 52.,
            Hz104 => 104.,
            Hz208 => 208.,
            Hz833 => 833.,
        }
    }

    pub fn gyro_odr(&self) -> ctrl2g::Odr {
        use ctrl2g::Odr;
        use Freq::*;

        match self {
            Hz26 => Odr::Hz26,
            Hz52 => Odr::Hz52,
            Hz104 => Odr::Hz104,
            Hz208 => Odr::Hz208,
            Hz833 => Odr::Hz833,
        }
    }

    pub fn accel_odr(&self) -> ctrl1xl::Odr_Xl {
        use ctrl1xl::Odr_Xl as Odr;
        use Freq::*;

        match self {
            Hz26 => Odr::Hz26,
            Hz52 => Odr::Hz52,
            Hz104 => Odr::Hz104,
            Hz208 => Odr::Hz208,
            Hz833 => Odr::Hz833,
        }
    }

    pub fn accel_bdr(&self) -> fifoctrl::BdrXl {
        use fifoctrl::BdrXl as Odr;
        use Freq::*;

        match self {
            Hz26 => Odr::Hz26,
            Hz52 => Odr::Hz52,
            Hz104 => Odr::Hz104,
            Hz208 => Odr::Hz208,
            Hz833 => Odr::Hz833,
        }
    }

    pub fn gyro_bdr(&self) -> fifoctrl::BdrGy {
        use fifoctrl::BdrGy as Odr;
        use Freq::*;

        match self {
            Hz26 => Odr::Hz26,
            Hz52 => Odr::Hz52,
            Hz104 => Odr::Hz104,
            Hz208 => Odr::Hz208,
            Hz833 => Odr::Hz833,
        }
    }
}

const freq: Freq = Freq::Hz208;

#[entry]
fn main() -> ! {
    unsafe {
        // Set the clock frequency.
        halc::am_hal_clkgen_control(
            halc::am_hal_clkgen_control_e_AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX,
            0 as *mut c_void,
        );

        // Set the default cache configuration
        halc::am_hal_cachectrl_config(&halc::am_hal_cachectrl_defaults);
        halc::am_hal_cachectrl_enable();

        // Configure the board for low power operation.
        halc::am_bsp_low_power_init();
    }

    let mut dp = hal::pac::Peripherals::take().unwrap();
    let core = hal::pac::CorePeripherals::take().unwrap();
    let mut delay = hal::delay::Delay::new(core.SYST, &mut dp.CLKGEN);

    let pins = hal::gpio::Pins::new(dp.GPIO);

    // set up serial
    let mut serial = hal::uart::new_48_49(dp.UART0, pins.tx0, pins.rx0, 1_000_000);

    uwriteln!(&mut serial, "C,hello from sfy-imu-usb!");
    uwriteln!(&mut serial, "C,setting up IOM and RTC.");
    delay.delay_ms(1_000u32);

    let mut i2c3 = i2c::I2c::new(dp.IOM3, pins.d6, pins.d7, i2c::Freq::F400kHz);

    // Set up RTC
    let mut rtc = hal::rtc::Rtc::new(dp.RTC, &mut dp.CLKGEN);
    rtc.set(
        &NaiveDate::from_ymd_opt(2020, 1, 1)
            .unwrap()
            .and_hms_opt(0, 0, 0)
            .unwrap(),
    ); // Now timestamps will be positive.
    rtc.enable();
    rtc.set_alarm_repeat(hal::rtc::AlarmRepeat::DeciSecond);
    rtc.enable_alarm();

    let mut led = pins.d19.into_push_pull_output();

    uwriteln!(&mut serial, "C,blinking to indicate start-up..");
    led.set_high().unwrap();

    uwriteln!(
        &mut serial,
        "C,giving sub-systems a couple of seconds to start up.."
    );
    delay.delay_ms(5_000u32);

    led.set_low().unwrap();

    uwriteln!(&mut serial, "C,setting up IMU..");
    let mut imu = Ism330Dhcx::new_with_address(&mut i2c3, 0x6a).unwrap();
    boot_imu(&mut i2c3, &mut imu).unwrap();
    disable_fifo(&mut i2c3, &mut imu).unwrap();

    // let mut waves = Waves::new(i2c3).unwrap();
    // waves
    //     .take_buf(
    //         now.map(|t| t.and_utc().timestamp_millis()).unwrap_or(0),
    //         position_time,
    //         lon,
    //         lat,
    //     )
    //     .unwrap(); // set timestamp.

    // info!("Enable IMU.");
    // waves.enable_fifo(&mut delay).unwrap();

    // uwriteln!(&mut serial, "C,enabling interrupts..");
    // unsafe {
    //     cortex_m::interrupt::enable();
    // }

    uwriteln!(&mut serial, "C,enable imu + fifo..");
    enable_fifo(&mut i2c3, &mut imu, &mut delay).unwrap();

    uwriteln!(&mut serial, "C,entering main loop..");

    loop {
        uwriteln!(&mut serial, "C,read available samples..");
        match read_samples(&mut i2c3, &mut imu) {
            Ok(samples) => {
                for (a, g) in samples {
                    // let g_rad = g.as_rad();
                    let g_dps = g.as_dps();

                    // let a_m_ss = a.as_m_ss();
                    let a_g = a.as_g();
                    uwriteln!(
                        &mut serial,
                        "A,{},{},{},G,{},{},{}",
                        Uf64(a_g[0], 9),
                        Uf64(a_g[1], 9),
                        Uf64(a_g[2], 9),
                        Uf64(g_dps[0], 9),
                        Uf64(g_dps[1], 9),
                        Uf64(g_dps[2], 9)
                    );
                }
            }
            Err(_) => {
                uwriteln!(&mut serial, "C,ERROR, restarting!");
                reset();
            }
        }
        serial.flush();
        led.toggle().unwrap();
    }
}

fn enable_fifo<E, IOM: WriteRead<Error = E> + Write<Error = E>>(
    i2c: &mut IOM,
    imu: &mut Ism330Dhcx,
    delay: &mut impl DelayMs<u16>,
) -> Result<(), E> {
    // Reset FIFO
    imu.fifoctrl.mode(i2c, fifoctrl::FifoMode::Bypass)?;
    imu.fifoctrl
        .set_accelerometer_batch_data_rate(i2c, freq.accel_bdr())?;
    imu.fifoctrl
        .set_gyroscope_batch_data_rate(i2c, freq.gyro_bdr())?;

    // Wait for FIFO to be cleared.
    delay.delay_ms(10);

    // clear status bits.
    imu.fifostatus.full(i2c)?;
    imu.fifostatus.overrun(i2c)?;
    imu.fifostatus.overrun_latched(i2c)?; // XXX: only necessary on this one.

    // Start FIFO. The FIFO will fill up and stop if it is not emptied fast enough.
    imu.fifoctrl.mode(i2c, fifoctrl::FifoMode::FifoMode)?;

    Ok(())
}

/// Disable FIFO mode (this also resets the FIFO).
fn disable_fifo<E, IOM: WriteRead<Error = E> + Write<Error = E>>(
    i2c: &mut IOM,
    imu: &mut Ism330Dhcx,
) -> Result<(), E> {
    imu.fifoctrl.mode(i2c, fifoctrl::FifoMode::Bypass)?;

    // Read FIFO status register to clear.
    let _fifo_full = imu.fifostatus.full(i2c)?;
    let _fifo_overrun = imu.fifostatus.overrun(i2c)?;
    let _fifo_overrun_latched = imu.fifostatus.overrun_latched(i2c)?;

    Ok(())
}

/// Returns iterator with all the currently available samples in the FIFO.
fn consume_fifo<'a, E, IOM: WriteRead<Error = E> + Write<Error = E>>(
    i2c: &'a mut IOM,
    imu: &'a mut Ism330Dhcx,
) -> Result<impl ExactSizeIterator<Item = Result<fifo::Value, E>> + 'a, E> {
    let n = imu.fifostatus.diff_fifo(i2c)?;
    Ok((0..n).map(|_| imu.fifo_pop(i2c)))
}

/// Read samples from IMU.
fn read_samples<'a, E: Debug, IOM: WriteRead<Error = E> + Write<Error = E>>(
    i2c: &'a mut IOM,
    imu: &'a mut Ism330Dhcx,
) -> Result<heapless::Vec<(ism330dhcx::AccelValue, ism330dhcx::GyroValue), 1024>, ImuError<E>> {
    use fifo::Value;

    let n = imu.fifostatus.diff_fifo(i2c)?;

    let fifo_full = imu.fifostatus.full(i2c)?;
    let fifo_overrun = imu.fifostatus.overrun(i2c)?;
    let fifo_overrun_latched = imu.fifostatus.overrun_latched(i2c)?;
    // defmt::trace!(
    //     "reading {} (fifo_full: {}, overrun: {}, overrun_latched: {}) sample pairs (buffer: {}/{})",
    //     n,
    //     fifo_full,
    //     fifo_overrun,
    //     fifo_overrun_latched,
    //     self.buf.len(),
    //     self.buf.capacity()
    // );

    // XXX: If any of these flags are true we need to reset the FIFO (and return an error from
    // this function), otherwise it will have stopped accumulating samples.
    if fifo_full || fifo_overrun || fifo_overrun_latched {
        // defmt::error!("IMU fifo overrun: fifo sz: {}, (fifo_full: {}, overrun: {}, overrun_latched: {}) (buffer: {}/{})", n, fifo_full, fifo_overrun, fifo_overrun_latched, self.buf.len(), self.buf.capacity());

        return Err(ImuError::FifoOverrun {
            fifo_full,
            overrun: fifo_overrun,
            latched: fifo_overrun_latched,
            samples: n,
        });
    }

    let n = n / 2;

    let mut samples = heapless::Vec::new();

    for _ in 0..n {
        if samples.is_full() {
            break;
        }

        let m1 = imu.fifo_pop(i2c)?;
        let m2 = imu.fifo_pop(i2c)?;

        let ga = match (m1, m2) {
            (Value::Gyro(g), Value::Accel(a)) => Some((g, a)),
            (Value::Accel(a), Value::Gyro(g)) => Some((g, a)),
            _ => None,
        };

        if let Some((g, a)) = ga {
            samples.push((a, g));
        } else {
            // defmt::error!("Bad sequence of samples in FIFO: {:?}, {:?}", m1, m2);
            return Err(ImuError::FifoBadSequence(m1, m2));
        }
    }

    let nn = imu.fifostatus.diff_fifo(i2c)?;
    // defmt::trace!("fifo length after read: {}", nn);

    Ok(samples)
}

fn boot_imu<E, IOM: WriteRead<Error = E> + Write<Error = E>>(
    i2c: &mut IOM,
    sensor: &mut Ism330Dhcx,
) -> Result<(), E> {
    // CTRL3_C
    sensor.ctrl3c.set_boot(i2c, true)?;
    sensor.ctrl3c.set_bdu(i2c, true)?;
    sensor.ctrl3c.set_if_inc(i2c, true)?;

    // CTRL9_XL
    // sensor.ctrl9xl.set_den_x(i2c, true)?;
    // sensor.ctrl9xl.set_den_y(i2c, true)?;
    // sensor.ctrl9xl.set_den_z(i2c, true)?;
    // sensor.ctrl9xl.set_device_conf(i2c, true)?;

    // CTRL1_XL
    sensor
        .ctrl1xl
        .set_accelerometer_data_rate(i2c, freq.accel_odr())?;

    // # Acceleration range
    //
    // Acceleration range is measured to up to 8 and 14 g in breaking waves. But much less in
    // normal waves.
    //
    // Important: Make sure that the range here matches the range specified in the serialization of
    // acceleration values.
    //
    // Feddersen, F., Andre Amador, Kanoa Pick, A. Vizuet, Kaden Quinn, Eric Wolfinger, J. H. MacMahan, and Adam Fincham. “The Wavedrifter: A Low-Cost IMU-Based Lagrangian Drifter to Observe Steepening and Overturning of Surface Gravity Waves and the Transition to Turbulence.” Coastal Engineering Journal, July 26, 2023, 1–14. https://doi.org/10.1080/21664250.2023.2238949.
    //
    // Sinclair, Alexandra. “FlowRider: A Lagrangian Float to Measure 3-D Dynamics of Plunging Breakers in the Surf Zone.” Journal of Coastal Research 293 (January 2014): 205–9. https://doi.org/10.2112/JCOASTRES-D-13-00014.1.

    // sensor
    //     .ctrl1xl
    //     .set_chain_full_scale(i2c, ctrl1xl::Fs_Xl::G4)?;

    // #[cfg(feature = "surf")]
    // sensor
    //     .ctrl1xl
    //     .set_chain_full_scale(i2c, ctrl1xl::Fs_Xl::G16)?;

    // #[cfg(feature = "ice")]
    sensor
        .ctrl1xl
        .set_chain_full_scale(i2c, ctrl1xl::Fs_Xl::G2)?;

    // uwrit!(
    //     "accelerometer range: {} g",
    //     sensor.ctrl1xl.chain_full_scale()
    // );
    // assert_eq!(sensor.ctrl1xl.chain_full_scale().g(), ACCEL_RANGE);

    sensor.ctrl1xl.set_lpf2_xl_en(i2c, true)?; // Use LPF2 filtering (cannot be used at the
                                               // same time as HP filter)
                                               // XL_HM_MODE is enabled by default (CTRL6C)

    // Accelerometer High-Pass filter: At least 30 seconds, preferably the same
    // as the gyro-scope (16 mHz).
    //
    // 0.016 = 208 Hz / X => X = 208 / 0.016 = 13000. The lowest is ODR / 800 which is 3.86
    //   seconds. This is too high, so we cannot use the built-in HP-filter.
    // sensor.ctrl8xl.set_hpcf(i2c, ctrl8xl::HPCF_XL::)

    // CTRL2_G
    sensor
        .ctrl2g
        .set_gyroscope_data_rate(i2c, freq.gyro_odr())?;

    // # Angular velocity range
    //
    // The angular velocity was measured to maximum 1000 dps with buoys travelling through a
    // breaking wave, with very seldomly values measured above 600 dps. We set it to 1000 in
    // surf-experiments, and lower in open-water experiments. For deployments in very quiet
    // areas, it would probably be best to use the lowest possible range (i.e. 125 for our
    // sensor).
    //
    // Important: Make sure that the range here matches the range specified in the serialization of
    // gyro values.
    //
    // Feddersen, F., Andre Amador, Kanoa Pick, A. Vizuet, Kaden Quinn, Eric Wolfinger, J. H. MacMahan, and Adam Fincham. “The Wavedrifter: A Low-Cost IMU-Based Lagrangian Drifter to Observe Steepening and Overturning of Surface Gravity Waves and the Transition to Turbulence.” Coastal Engineering Journal, July 26, 2023, 1–14. https://doi.org/10.1080/21664250.2023.2238949.
    // #[cfg(all(not(feature = "surf"), not(feature = "ice")))]
    // sensor
    //     .ctrl2g
    //     .set_chain_full_scale(i2c, ctrl2g::Fs::Dps500)?;

    // #[cfg(feature = "surf")]
    // sensor
    //     .ctrl2g
    //     .set_chain_full_scale(i2c, ctrl2g::Fs::Dps1000)?;

    sensor
        .ctrl2g
        .set_chain_full_scale(i2c, ctrl2g::Fs::Dps125)?;

    // defmt::info!("gyroscope range: {} dps", sensor.ctrl2g.chain_full_scale());
    // assert_eq!(sensor.ctrl2g.chain_full_scale().dps(), GYRO_RANGE);

    // CTRL7_G
    sensor.ctrl7g.set_g_hm_mode(i2c, true)?; // high-res mode on gyro (default is already on)

    // High-pass filter for gyro
    // sensor.ctrl7g.set_hpm_g(i2c, ctrl7g::Hpm_g::Hpmg16)?; // HPF at 16mHz (62.5
    //                                                       // seconds)
    // sensor.ctrl7g.set_hp_en_g(i2c, true)?;

    // Both the gyro and accelerometer is low-pass filtered on-board:
    //
    // Gyro: LPF2 at 66.8 Hz when ODR = 208 Hz (not configurable)
    // Accel: default is ODR/2 => 104 Hz.

    Ok(())
}

fn reset() -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}

// #[cfg(not(feature = "host-tests"))]
// #[allow(non_snake_case)]
// #[interrupt]
// fn RTC() {
//     // FIFO size of IMU is 512 samples (uncompressed), sample rate at IMU is 208 Hz. So we
//     // need to empty FIFO at atleast (208 / 512) Hz = 0.406 Hz or every 2.46 s.

//     // Clear RTC interrupt
//     unsafe {
//         (*(hal::pac::RTC::ptr()))
//             .intclr
//             .write(|w| w.alm().set_bit());
//     }

// }

#[allow(non_snake_case)]
#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}

#[cfg(feature = "deploy")]
#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}
