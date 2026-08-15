//! PCA9685 PWM controller access.
//!
//! Write operations keep all inactive channels in their individual `full-off`
//! state. The PCA9685 global `ALL_LED_*` registers are neutralized only while
//! the oscillator is stopped: they must not be used as a runtime output gate.

use anyhow::{Context, Result};
use i2cdev::{core::I2CDevice, linux::LinuxI2CDevice};
use std::path::Path;

const MODE1: u8 = 0x00;
const MODE2: u8 = 0x01;
const PRESCALE: u8 = 0xfe;
const ALL_LED_ON_L: u8 = 0xfa;
const ALL_LED_ON_H: u8 = 0xfb;
const ALL_LED_OFF_L: u8 = 0xfc;
const ALL_LED_OFF_H: u8 = 0xfd;
const MODE1_RESTART: u8 = 0x80;
const MODE1_AUTO_INCREMENT: u8 = 0x20;
const MODE1_SLEEP: u8 = 0x10;
const FULL_OFF: u8 = 0x10;
const LED0_ON_L: u8 = 0x06;
const CHANNEL_REGISTER_STRIDE: u8 = 4;
const CHANNEL_COUNT: u8 = 16;
const OSCILLATOR_HZ: f64 = 25_000_000.0;
const PWM_STEPS: f64 = 4096.0;
/// DS3218's documented control range.
pub const NOMINAL_MIN_PULSE_US: u16 = 500;
pub const NOMINAL_MAX_PULSE_US: u16 = 2500;
// Exploratory bounds for an assembled stand. These are not servo ratings;
// `rubik-servo-calibrate` requires a separate explicit acknowledgement before
// it can use them.
const ABSOLUTE_MIN_CALIBRATION_PULSE_US: u16 = 300;
const ABSOLUTE_MAX_CALIBRATION_PULSE_US: u16 = 2800;

pub struct Pca9685 {
    device: LinuxI2CDevice,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pca9685Status {
    pub mode1: u8,
    pub mode2: u8,
    pub prescale: u8,
}

impl Pca9685Status {
    pub const fn sleeping(self) -> bool {
        self.mode1 & MODE1_SLEEP != 0
    }

    pub const fn auto_increment(self) -> bool {
        self.mode1 & MODE1_AUTO_INCREMENT != 0
    }

    pub const fn all_call_enabled(self) -> bool {
        self.mode1 & 0x01 != 0
    }

    pub const fn push_pull_output(self) -> bool {
        self.mode2 & 0x04 != 0
    }

    /// Estimated PWM frequency using PCA9685's nominal 25 MHz oscillator.
    pub fn pwm_hz(self) -> f64 {
        OSCILLATOR_HZ / (PWM_STEPS * (f64::from(self.prescale) + 1.0))
    }
}

impl Pca9685 {
    /// Opens a single PCA9685 by Linux I²C device node and 7-bit address.
    pub fn open(device_path: &Path, address: u16) -> Result<Self> {
        let device = LinuxI2CDevice::new(device_path, address).with_context(|| {
            format!(
                "failed to open PCA9685 at {} address 0x{address:02x}",
                device_path.display()
            )
        })?;
        Ok(Self { device })
    }

    /// Reads status registers only; this method never writes to the controller.
    pub fn status(&mut self) -> Result<Pca9685Status> {
        let mode1 = self
            .device
            .smbus_read_byte_data(MODE1)
            .context("failed to read PCA9685 MODE1")?;
        let mode2 = self
            .device
            .smbus_read_byte_data(MODE2)
            .context("failed to read PCA9685 MODE2")?;
        let prescale = self
            .device
            .smbus_read_byte_data(PRESCALE)
            .context("failed to read PCA9685 PRESCALE")?;
        Ok(Pca9685Status {
            mode1,
            mode2,
            prescale,
        })
    }

    /// Sets the PWM period and leaves every PCA9685 output fully off.
    ///
    /// The controller is kept asleep while its prescaler is changed. All
    /// individual channels are set to full-off before it wakes.
    pub fn initialize_safe_pwm(&mut self, frequency_hz: f64) -> Result<Pca9685Status> {
        let prescale = prescale_for_hz(frequency_hz)?;
        let before = self.status()?;
        let sleeping_mode = (before.mode1 & !MODE1_RESTART) | MODE1_SLEEP;

        self.write_byte(MODE1, sleeping_mode, "put PCA9685 into sleep")?;
        self.clear_global_all_led_control()?;
        self.force_all_channels_off()?;
        self.write_byte(PRESCALE, prescale, "set PCA9685 PWM prescale")?;

        let awake_mode = (before.mode1 | MODE1_AUTO_INCREMENT) & !(MODE1_SLEEP | MODE1_RESTART);
        self.write_byte(MODE1, awake_mode, "wake PCA9685")?;
        std::thread::sleep(std::time::Duration::from_micros(500));
        self.write_byte(
            MODE1,
            awake_mode | MODE1_RESTART,
            "restart PCA9685 oscillator",
        )?;

        self.status()
    }

    fn write_byte(&mut self, register: u8, value: u8, operation: &str) -> Result<()> {
        self.device
            .smbus_write_byte_data(register, value)
            .with_context(|| format!("failed to {operation}"))
    }

    /// Emits a PWM pulse on exactly one channel for a bounded duration.
    ///
    /// All channels are forced off before the selected channel is configured.
    /// They are forced off again before the method returns. The caller must
    /// keep the duration short during mechanical calibration.
    pub fn pulse_channel_for(
        &mut self,
        channel: u8,
        pulse_us: u16,
        duration: std::time::Duration,
    ) -> Result<()> {
        self.pulse_channels_for(&[(channel, pulse_us)], duration)
    }

    /// Emits pulses on a set of channels simultaneously for a bounded duration.
    ///
    /// All other channels remain full-off. This is used for coordinated stand
    /// actions such as opening or gripping with all four rails.
    pub fn pulse_channels_for(
        &mut self,
        channels: &[(u8, u16)],
        duration: std::time::Duration,
    ) -> Result<()> {
        if channels.is_empty() {
            anyhow::bail!("at least one PCA9685 channel is required");
        }
        let mut configured = Vec::with_capacity(channels.len());
        for &(channel, pulse_us) in channels {
            if channel >= CHANNEL_COUNT {
                anyhow::bail!("PCA9685 channel must be 0..15, got {channel}");
            }
            if configured.iter().any(|&(existing, _)| existing == channel) {
                anyhow::bail!("PCA9685 channel {channel} was specified more than once");
            }
            if !(ABSOLUTE_MIN_CALIBRATION_PULSE_US..=ABSOLUTE_MAX_CALIBRATION_PULSE_US)
                .contains(&pulse_us)
            {
                anyhow::bail!(
                    "pulse must be {ABSOLUTE_MIN_CALIBRATION_PULSE_US}..={ABSOLUTE_MAX_CALIBRATION_PULSE_US} us, got {pulse_us}"
                );
            }
            configured.push((channel, pulse_us));
        }

        let status = self.status()?;
        if status.sleeping() {
            anyhow::bail!("PCA9685 is asleep; run rubik-servo-init first");
        }
        if !(40.0..=60.0).contains(&status.pwm_hz()) {
            anyhow::bail!(
                "PCA9685 PWM rate is {:.3} Hz, expected 40..=60 Hz; run rubik-servo-init --pwm-hz 50 first",
                status.pwm_hz()
            );
        }

        self.prepare_individual_channel_control(status.mode1)?;
        self.force_all_channels_off()?;

        for &(channel, pulse_us) in &configured {
            let ticks = pulse_ticks(pulse_us, status.pwm_hz())?;
            let base = LED0_ON_L + channel * CHANNEL_REGISTER_STRIDE;
            self.write_byte(base, 0, "set selected channel on-time low")?;
            self.write_byte(base + 1, 0, "set selected channel on-time high")?;
            self.write_byte(
                base + 2,
                (ticks & 0xff) as u8,
                "set selected channel off-time low",
            )?;
        }
        // The last register write enables each configured channel. Other
        // channels remain full-off, so no unrequested servo can move.
        for &(channel, pulse_us) in &configured {
            let ticks = pulse_ticks(pulse_us, status.pwm_hz())?;
            let base = LED0_ON_L + channel * CHANNEL_REGISTER_STRIDE;
            self.write_byte(
                base + 3,
                (ticks >> 8) as u8,
                "set selected channel off-time high",
            )?;
        }
        std::thread::sleep(duration);
        self.force_all_channels_off()
    }

    fn force_all_channels_off(&mut self) -> Result<()> {
        for channel in 0..CHANNEL_COUNT {
            let off_high = LED0_ON_L + channel * CHANNEL_REGISTER_STRIDE + 3;
            self.write_byte(off_high, FULL_OFF, "disable PCA9685 channel")?;
        }
        Ok(())
    }

    fn clear_global_all_led_control(&mut self) -> Result<()> {
        self.write_byte(ALL_LED_ON_L, 0, "clear global PCA9685 on-time low")?;
        self.write_byte(ALL_LED_ON_H, 0, "clear global PCA9685 on-time high")?;
        self.write_byte(ALL_LED_OFF_L, 0, "clear global PCA9685 off-time low")?;
        self.write_byte(ALL_LED_OFF_H, 0, "clear global PCA9685 off-time high")
    }

    fn prepare_individual_channel_control(&mut self, mode1: u8) -> Result<()> {
        let sleeping_mode = (mode1 & !MODE1_RESTART) | MODE1_SLEEP;
        self.write_byte(MODE1, sleeping_mode, "put PCA9685 into sleep")?;
        self.clear_global_all_led_control()?;
        self.force_all_channels_off()?;

        let awake_mode = (mode1 | MODE1_AUTO_INCREMENT) & !(MODE1_SLEEP | MODE1_RESTART);
        self.write_byte(MODE1, awake_mode, "wake PCA9685")?;
        std::thread::sleep(std::time::Duration::from_micros(500));
        self.write_byte(
            MODE1,
            awake_mode | MODE1_RESTART,
            "restart PCA9685 oscillator",
        )
    }
}

fn prescale_for_hz(frequency_hz: f64) -> Result<u8> {
    if !frequency_hz.is_finite() || frequency_hz <= 0.0 {
        anyhow::bail!("PWM frequency must be a positive finite number, got {frequency_hz}");
    }

    let raw = (OSCILLATOR_HZ / (PWM_STEPS * frequency_hz) - 1.0).round();
    if !(3.0..=255.0).contains(&raw) {
        anyhow::bail!(
            "PWM frequency {frequency_hz:.3} Hz is outside PCA9685 range {:.3}..={:.3} Hz",
            OSCILLATOR_HZ / (PWM_STEPS * 256.0),
            OSCILLATOR_HZ / (PWM_STEPS * 4.0),
        );
    }
    Ok(raw as u8)
}

fn pulse_ticks(pulse_us: u16, pwm_hz: f64) -> Result<u16> {
    let ticks = (f64::from(pulse_us) * pwm_hz * PWM_STEPS / 1_000_000.0).round();
    if !(1.0..PWM_STEPS).contains(&ticks) {
        anyhow::bail!("pulse {pulse_us} us does not fit one PWM period at {pwm_hz:.3} Hz");
    }
    Ok(ticks as u16)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_observed_default_registers() {
        let status = Pca9685Status {
            mode1: 0x11,
            mode2: 0x04,
            prescale: 0x1e,
        };
        assert!(status.sleeping());
        assert!(status.all_call_enabled());
        assert!(!status.auto_increment());
        assert!(status.push_pull_output());
        assert!((status.pwm_hz() - 196.89).abs() < 0.01);
    }

    #[test]
    fn calculates_50_hz_prescale() {
        assert_eq!(prescale_for_hz(50.0).unwrap(), 121);
        let status = Pca9685Status {
            mode1: 0,
            mode2: 0,
            prescale: 121,
        };
        assert!((status.pwm_hz() - 50.029).abs() < 0.001);
    }

    #[test]
    fn rejects_out_of_range_frequency() {
        assert!(prescale_for_hz(0.0).is_err());
        assert!(prescale_for_hz(2_000.0).is_err());
    }

    #[test]
    fn calculates_pulse_ticks_at_50_hz() {
        assert_eq!(pulse_ticks(1_500, 50.029).unwrap(), 307);
    }
}
