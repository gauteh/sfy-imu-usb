#![no_std]
#![no_main]

// we use this for defs of sinf etc.
extern crate cmsis_dsp;

use ambiq_hal::{self as hal, prelude::*};
use chrono::NaiveDate;
use core::cell::RefCell;
use core::fmt::Write as _;
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
    i2c::{Read, Write},
};

use git_version::git_version;
use hal::{i2c, pac::interrupt};

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
    #[cfg(not(feature = "deploy"))]
    let mut led = pins.d19.into_push_pull_output(); // d14 on redboard_artemis

    // set up serial
    let serial = hal::uart::new_48_49(dp.UART0, pins.tx0, pins.rx0, 115000);

    println!(
        "hello from sfy (v{}) (sn: {})!",
        git_version!(),
        sfy::note::BUOYSN
    );

    info!("Setting up IOM and RTC.");
    delay.delay_ms(1_000u32);

    let i2c3 = i2c::I2c::new(dp.IOM3, pins.d6, pins.d7, i2c::Freq::F400kHz);

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

    info!("Blinking to indicate start-up.");
    led.set_high().unwrap();

    info!("Giving subsystems a couple of seconds to boot..");
    delay.delay_ms(5_000u32);

    led.set_low().unwrap();

    info!("Setting up IMU..");
    let mut waves = Waves::new(i2c3).unwrap();
    waves
        .take_buf(
            now.map(|t| t.and_utc().timestamp_millis()).unwrap_or(0),
            position_time,
            lon,
            lat,
        )
        .unwrap(); // set timestamp.

    info!("Enable IMU.");
    waves.enable_fifo(&mut delay).unwrap();

    defmt::info!("Enable interrupts");
    unsafe {
        cortex_m::interrupt::enable();
    }

    info!("Entering main loop");

    loop {}
}

fn reset<I: Read + Write>(note: &mut Notecarrier<I>, delay: &mut impl DelayMs<u16>) -> ! {
    cortex_m::interrupt::disable();

    warn!("Resetting in 3 seconds..");
    delay.delay_ms(3_000u16);

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
