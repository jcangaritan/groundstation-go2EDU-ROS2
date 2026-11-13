// accelerometer_sampler.hpp
//
// WHAT IT DOES
//   Decides WHICH accelerometer data can be trusted to measure gravity. It does not compute any
//   rotation: it only collects a window of samples while the robot is still, rejects the ones that
//   look wrong, and hands out the window mean when the window is valid.
//
// PIPELINE (one call to addSample() per IMU message)
//   Step A. Per-sample gates      -> motion (gyro) and wrong magnitude |a| discard the sample and
//                                    restart the window.
//   Step B. Accumulate            -> running sums of a and a^2 (per axis).
//   Step C. Window full?          -> wait until n_samples samples were accepted in a row.
//   Step D. Window gate           -> per-axis standard deviation must be small, otherwise the whole
//                                    window is discarded and restarted.
//   Step E. Ready                 -> mean() is the accelerometer vector to align with gravity.
//
// DESIGN NOTES
//   * Pure C++ (Eigen only). No ROS here, so it can be unit-tested without ROS. Logging is optional
//     and goes through a callback; the ROS node passes a lambda that calls RCLCPP_INFO.
//   * Units: a is whatever the sensor gives (g for the Livox, m/s^2 for others). Set expected_norm
//     to match (1.0 or 9.81). Only the direction of the mean matters downstream.
//   * Convention: at rest the accelerometer measures the reaction to gravity, so a points UP.
#pragma once

#include <Eigen/Dense>

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <functional>
#include <string>
#include <utility>

namespace gravity {

enum class SampleStatus {
  Collecting,     // sample accepted, window not full yet
  RejectedGyro,   // sample discarded: the sensor is moving (gyro gate). Window restarted.
  RejectedNorm,   // sample discarded: |a| is not close to expected_norm. Window restarted.
  RejectedNoise,  // window was full but too noisy. Window discarded and restarted.
  Ready           // window full and valid: mean() can be used
};

class AccelerometerSampler {
 public:
  struct Params {
    double expected_norm = 1.0;   // 1.0 if accel is in g (Livox), 9.81 if in m/s^2
    double norm_tol = 0.05;       // allowed relative error of |a| w.r.t. expected_norm
    double max_gyro = 0.05;       // rad/s; <= 0 disables the gyro gate (accelerometer-only)
    double max_std = 0.01;        // max per-axis std of the window, relative to expected_norm
    std::size_t n_samples = 400;  // window length (2 s at 200 Hz)
  };

  // Counters kept across restarts, useful to understand WHY nothing is accepted.
  struct Stats {
    std::size_t samples = 0;   // samples seen
    std::size_t rej_gyro = 0;  // samples rejected by the gyro gate
    std::size_t rej_norm = 0;  // samples rejected by the magnitude gate
    std::size_t rej_std = 0;   // full windows rejected for noise
    double last_norm = 0.0;    // last |a|
    double last_gyro = 0.0;    // last |gyro|
    double last_std = 0.0;     // max per-axis std of the last full window
  };

  using LogFn = std::function<void(const std::string&)>;

  AccelerometerSampler() = default;
  explicit AccelerometerSampler(const Params& p, LogFn log = nullptr)
      : p_(p), log_(std::move(log)) {}

  // Feed one IMU sample (accel and gyro as given by the sensor).
  SampleStatus addSample(const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro) {
    if (ready_) return SampleStatus::Ready;

    // -- Step A: per-sample gates ------------------------------------------------------------
    ++stats_.samples;
    stats_.last_norm = accel.norm();
    stats_.last_gyro = gyro.norm();

    const bool moving = p_.max_gyro > 0.0 && stats_.last_gyro > p_.max_gyro;
    const bool bad_norm =
        std::abs(stats_.last_norm - p_.expected_norm) > p_.norm_tol * p_.expected_norm;
    if (moving || bad_norm) {
      if (moving) ++stats_.rej_gyro;
      if (bad_norm) ++stats_.rej_norm;
      restartWindow();
      return moving ? SampleStatus::RejectedGyro : SampleStatus::RejectedNorm;
    }

    // -- Step B: accumulate -------------------------------------------------------------------
    sum_ += accel;
    sum_sq_ += accel.cwiseProduct(accel);
    ++n_;

    // -- Step C: is the window full? ----------------------------------------------------------
    if (n_ < p_.n_samples) return SampleStatus::Collecting;

    // -- Step D: window gate (noise) ----------------------------------------------------------
    stats_.last_std = stdDev().maxCoeff();
    if (stats_.last_std > p_.max_std * p_.expected_norm) {
      ++stats_.rej_std;
      log("window rejected: too noisy", stats_.last_std, p_.max_std * p_.expected_norm);
      restartWindow();
      return SampleStatus::RejectedNoise;
    }

    // -- Step E: ready ------------------------------------------------------------------------
    ready_ = true;
    log("window ready: std below limit", stats_.last_std, p_.max_std * p_.expected_norm);
    return SampleStatus::Ready;
  }

  bool ready() const { return ready_; }
  std::size_t count() const { return n_; }          // samples in the current window
  const Stats& stats() const { return stats_; }
  const Params& params() const { return p_; }

  // Mean of the current window (the vector to be aligned with gravity once ready()).
  Eigen::Vector3d mean() const {
    return n_ ? Eigen::Vector3d(sum_ / static_cast<double>(n_)) : Eigen::Vector3d::Zero();
  }

  // Per-axis sample standard deviation of the current window.
  Eigen::Vector3d stdDev() const {
    if (n_ < 2) return Eigen::Vector3d::Zero();
    const double n = static_cast<double>(n_);
    const Eigen::Vector3d var = (sum_sq_ - sum_.cwiseProduct(sum_) / n) / (n - 1.0);
    return var.cwiseMax(0.0).cwiseSqrt();
  }

  // Forget everything (also the "ready" state). Statistics are kept.
  void reset() {
    restartWindow();
    ready_ = false;
  }

 private:
  void restartWindow() {
    sum_.setZero();
    sum_sq_.setZero();
    n_ = 0;
  }

  void log(const char* msg, double value, double limit) const {
    if (!log_) return;
    char buf[160];
    std::snprintf(buf, sizeof(buf), "[SAMPLING IMU] %s (std=%.5f, limit=%.5f, n=%zu)", msg, value,
                  limit, n_);
    log_(buf);
  }

  Params p_;
  LogFn log_;
  Stats stats_;
  Eigen::Vector3d sum_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_sq_ = Eigen::Vector3d::Zero();
  std::size_t n_ = 0;
  bool ready_ = false;
};

}  // namespace gravity
