//! A small PID controller library.
//!
//! This crate implements the classic independent PID formulation.
//!
//! # Introduction
//!
//! PID controllers are an integral part of control systems, and provide a way to
//! perform error correction. It's used to control things like throughput or
//! resource allocation: as the resource approaches capacity, the returned
//! correction decreases. And because it is aware of a time factor, it can deal
//! with rapid changes as well.
//!
//! # Loop Tuning
//!
//! However PID controllers are not a silver bullet: they are a tool in a wider
//! toolbox. To maximally benefit from them they need to be tuned to the
//! workload. This is done through three parameters: `proportional_gain`,
//! `integral_gain` and `derivative_gain`. Automated algorithms exist to tune
//! these parameters based on sample workloads, but those are out of scope for
//! this crate.
//!
//! [Read more on loop tuning](https://en.wikipedia.org/wiki/PID_controller#Loop_tuning).
//!
//! # No-std support
//!
//! `#[no_std]` support can be enabled by disabling the default crate-level
//! features. This disables the `Controller::update` method which automatically
//! calculates the time elapsed. Instead use the `Controller::update_elapsed`
//! method which takes an externally calculated `Duration`.
//!
//! # Examples
//!
//! ```no_run
//! use pid_lite::Controller;
//! use std::thread;
//! use std::time::Duration;
//!
//! let target = 80.0;
//! let mut controller = Controller::new(target, 0.25, 0.01, 0.01);
//!
//! loop {
//!     let correction = controller.update(measure());
//!     apply_correction(correction);
//!     thread::sleep(Duration::from_secs(1));
//! }
//! # fn measure() -> f64 { todo!() }
//! # fn apply_correction(_: f64) { todo!() }
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]
#![deny(missing_debug_implementations, nonstandard_style)]
#![warn(missing_docs, future_incompatible, unreachable_pub, rust_2018_idioms)]

use core::time::Duration;
#[cfg(feature = "std")]
use std::time::Instant;

/// PID controller
///
/// The `target` param sets the value we want to reach. The
/// `proportional_gain`, `integral_gain` and `derivative_gain` parameters are all
/// tuning parameters.
///
/// # Examples
///
/// ```no_run
/// use pid_lite::Controller;
/// use std::thread;
/// use std::time::Duration;
///
/// let target = 80.0;
/// let mut controller = Controller::new(target, 0.5, 0.1, 0.2);
///
/// loop {
///     let correction = controller.update(measure());
///     apply_correction(correction);
///     thread::sleep(Duration::from_secs(1));
/// }
/// # fn measure() -> f64 { todo!() }
/// # fn apply_correction(_: f64) { todo!() }
/// ```

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Controller {
    target: f64,

    proportional_gain: f64,
    integral_gain: f64,
    derivative_gain: f64,

    error_sum: f64,
    last_error: f64,
    #[cfg(feature = "std")]
    last_instant: Option<Instant>,
}

impl Controller {
    /// Create a new instance of `Controller`.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![allow(unused_assignments)]
    /// use pid_lite::Controller;
    ///
    /// let target = 80.0;
    /// let mut controller = Controller::new(target, 0.20, 0.02, 0.04);
    /// ```
    pub const fn new(
        target: f64,
        proportional_gain: f64,
        integral_gain: f64,
        derivative_gain: f64,
    ) -> Self {
        Self {
            target,
            proportional_gain,
            integral_gain,
            derivative_gain,
            error_sum: 0.0,
            last_error: 0.0,
            #[cfg(feature = "std")]
            last_instant: None,
        }
    }

    /// Get the target.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![allow(unused_assignments)]
    /// use pid_lite::Controller;
    ///
    /// let target = 80.0;
    /// let mut controller = Controller::new(target, 0.20, 0.02, 0.04);
    /// assert_eq!(controller.target(), 80.0);
    /// ```
    pub const fn target(&self) -> f64 {
        self.target
    }

    /// Set the target.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![allow(unused_assignments)]
    /// use pid_lite::Controller;
    ///
    /// let target = 80.0;
    /// let mut controller = Controller::new(target, 0.20, 0.02, 0.04);
    /// controller.set_target(60.0);
    /// assert_eq!(controller.target(), 60.0);
    /// ```
    pub fn set_target(&mut self, target: f64) {
        self.target = target;
    }

    /// Set the proportional gain
    pub fn set_proportional_gain(&mut self, proportional_gain: f64) { self.proportional_gain = proportional_gain; }

    /// Set the integral gain
    pub fn set_integral_gain(&mut self, integral_gain: f64) { self.integral_gain = integral_gain; }

    /// Set the derivative gain
    pub fn set_derivative_gain(&mut self, derivative_gain: f64) { self.derivative_gain = derivative_gain; }

    /// Push an entry into the controller.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![allow(unused_assignments)]
    /// use pid_lite::Controller;
    ///
    /// let target = 80.0;
    /// let mut controller = Controller::new(target, 0.0, 0.0, 0.0);
    /// assert_eq!(controller.update(60.0), 0.0);
    /// ```
    ///
    /// # Panics
    ///
    /// This function may panic if the `time_delta` in millis no longer fits in
    /// an `f64`. This limit can be encountered when the PID controller is updated on the scale of
    /// hours, rather than on the scale of minutes to milliseconds.
    #[cfg(feature = "std")]
    #[must_use = "A PID controller does nothing if the correction is not applied"]
    pub fn update(&mut self, current_value: f64) -> f64 {
        let now = Instant::now();
        let elapsed = match self.last_instant {
            Some(last_time) => now.duration_since(last_time),
            None => Duration::from_millis(1),
        };
        self.last_instant = Some(now);
        self.update_elapsed(current_value, elapsed)
    }

    /// Push an entry into the controller with a time delta since the last update.
    ///
    /// The `time_delta` value will be rounded down to the closest millisecond
    /// with a minimum of 1 millisecond.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![allow(unused_assignments)]
    /// use pid_lite::Controller;
    /// use std::time::Duration;
    ///
    /// let target = 80.0;
    /// let mut controller = Controller::new(target, 0.5, 0.1, 0.2);
    /// let dur = Duration::from_millis(2);
    /// assert_eq!(controller.update_elapsed(60.0, dur), 16.0);
    /// ```
    ///
    /// # Panics
    ///
    /// This function may panic if the `time_delta` in millis no longer fits in
    /// an `f64`. This limit can be encountered when the PID controller is updated on the scale of
    /// hours, rather than on the scale of minutes to milliseconds.
    #[must_use = "A PID controller does nothing if the correction is not applied"]
    pub fn update_elapsed(&mut self, current_value: f64, elapsed: Duration) -> f64 {
        let elapsed = (elapsed.as_millis() as f64).max(1.0);

        let error = self.target - current_value;
        let error_delta = (error - self.last_error) / elapsed;
        self.error_sum = self.error_sum + error * elapsed;
        self.last_error = error;

        let p = self.proportional_gain * error;
        let i = self.integral_gain * self.error_sum;
        let d = self.derivative_gain * error_delta;

        p + i + d
    }

    /// Reset the internal state.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![allow(unused_assignments)]
    /// use pid_lite::Controller;
    /// use std::time::Duration;
    ///
    /// let target = 80.0;
    /// let mut controller = Controller::new(target, 0.0, 0.0, 0.0);
    /// let dur = Duration::from_secs(2);
    /// let correction = controller.update_elapsed(60.0, dur);
    ///
    /// controller.reset();
    /// ```
    pub fn reset(&mut self) {
        self.reset_inner();
    }

    #[cfg(feature = "std")]
    fn reset_inner(&mut self) {
        self.error_sum = 0.0;
        self.last_error = 0.0;
        self.last_instant = None;
    }

    #[cfg(not(feature = "std"))]
    pub fn reset_inner(&mut self) {
        self.error_sum = 0.0;
        self.last_error = 0.0;
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn base_correction() {
        let target = 80.0;
        let mut controller = Controller::new(target, 0.5, 0.1, 0.2);
        let dur = Duration::from_millis(4);
        assert_eq!(controller.update_elapsed(60.0, dur), 19.0);
    }

    #[test]
    #[cfg(feature = "std")]
    fn no_correction() {
        let target = 80.0;
        let mut controller = Controller::new(target, 0.0, 0.0, 0.0);
        assert_eq!(controller.update(60.0), 0.0);
    }
}
