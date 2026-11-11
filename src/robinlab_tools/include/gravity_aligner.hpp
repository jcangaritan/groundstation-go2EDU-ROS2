// gravity_aligner.hpp - simple static gravity alignment (roll/pitch) with Sophus.
//
// Convention
//   a_body : accelerometer reading at rest. It measures the *reaction* to gravity,
//            so it points UP (a ~ +z when the sensor is level), in the sensor frame.
//   R_wb   : rotation body -> world, with world z up and yaw fixed to 0.
//            R_wb * a_body is parallel to +z  (equivalently, R_wb * g_body = [0,0,-1]).
//
// Yaw is not observable from gravity: any rotation about the vertical also fits.
// GravityAligner uses the minimal (geodesic) rotation: R_wb = eps (+) delta_theta = Exp(delta_theta),
// which has no twist about the vertical. rollPitchFromAccel() gives the ZYX alternative with yaw = 0
// (R = Ry(pitch) * Rx(roll)); both share the same tilt and differ only by a small yaw (~ roll*pitch/2).
#pragma once

#include <Eigen/Dense>
#include <sophus/so3.hpp>

#include <cmath>
#include <cstddef>

namespace gravity {

// Roll/pitch (yaw = 0) from one accelerometer vector (any units, not necessarily normalised).
inline Sophus::SO3d rollPitchFromAccel(const Eigen::Vector3d& a) {
  const double roll = std::atan2(a.y(), a.z());
  const double pitch = std::atan2(-a.x(), std::hypot(a.y(), a.z()));
  return Sophus::SO3d::exp(Eigen::Vector3d(0.0, pitch, 0.0)) *
         Sophus::SO3d::exp(Eigen::Vector3d(roll, 0.0, 0.0));
}

// delta_theta in T_eps SO(3) = so(3) ~ R^3: the rotation vector that takes a/|a| to +z.
//   n = (a x z) / |a x z|,  theta = atan2(|a x z|, a . z),  delta_theta = theta * n
// (a x z gives sin(theta)*n and a . z gives cos(theta); atan2 keeps it stable near 0 and pi).
inline Eigen::Vector3d gravityIncrement(const Eigen::Vector3d& a_in) {
  const Eigen::Vector3d a = a_in.normalized();
  const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d axis = a.cross(up);
  const double s = axis.norm();
  const double c = a.dot(up);
  if (s < 1e-12) {
    if (c > 0.0) return Eigen::Vector3d::Zero();  // already aligned
    return M_PI * a.unitOrthogonal();             // upside down: 180 deg about any perpendicular axis
  }
  return axis / s * std::atan2(s, c);
}

// R = eps (+) delta_theta = I * Exp(delta_theta) = Exp(delta_theta)   (eps = identity, so left/right plus coincide)
// Minimal rotation: no twist about the vertical, so yaw is whatever that gives (not ZYX yaw = 0).
inline Sophus::SO3d minimalRotationToUp(const Eigen::Vector3d& a) {
  return Sophus::SO3d() * Sophus::SO3d::exp(gravityIncrement(a));
}

// Collects a window of samples while the sensor is still, then returns R_wb.
// Any sample that looks like motion (gyro too high, |a| far from 1 g) restarts the window.
class GravityAligner {
 public:
  struct Params {
    double expected_norm = 1.0;  // 1.0 if accel is in g (Livox), 9.81 if in m/s^2
    double norm_tol = 0.05;      // allowed relative error of |a| w.r.t. expected_norm
    double max_gyro = 0.05;      // rad/s
    double max_std = 0.01;       // per-axis std of a, relative to expected_norm
    std::size_t n_samples = 400; // window length (2 s at 200 Hz)
  };

  GravityAligner() = default;
  explicit GravityAligner(const Params& p) : p_(p) {}

  void reset() {
    sum_.setZero();
    sum_sq_.setZero();
    n_ = 0;
    done_ = false;
  }

  // Feed one IMU sample. Returns true once the alignment is available (and keeps returning true until reset()).
  bool addSample(const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro) {
    if (done_) return true;

    const bool moving = gyro.norm() > p_.max_gyro;
    const bool bad_norm =
        std::abs(accel.norm() - p_.expected_norm) > p_.norm_tol * p_.expected_norm;
    if (moving || bad_norm) {
      reset();
      return false;
    }

    sum_ += accel;
    sum_sq_ += accel.cwiseProduct(accel);
    if (++n_ < p_.n_samples) return false;

    // Window full: check that it was quiet enough before trusting the mean.
    if (stdAccel().maxCoeff() > p_.max_std * p_.expected_norm) {
      reset();
      return false;
    }
    delta_theta_ = gravityIncrement(meanAccel());       // delta_theta in T_eps SO(3)
    R_wb_ = Sophus::SO3d() * Sophus::SO3d::exp(delta_theta_);  // R = eps (+) delta_theta
    done_ = true;
    return true;
  }

  bool done() const { return done_; }
  const Sophus::SO3d& R_wb() const { return R_wb_; }
  const Eigen::Vector3d& deltaTheta() const { return delta_theta_; }  // rotation vector of the correction
  std::size_t count() const { return n_; }

  Eigen::Vector3d meanAccel() const { return n_ ? Eigen::Vector3d(sum_ / double(n_)) : Eigen::Vector3d::Zero(); }

  // Per-axis sample standard deviation (diagnostic).
  Eigen::Vector3d stdAccel() const {
    if (n_ < 2) return Eigen::Vector3d::Zero();
    const double n = double(n_);
    Eigen::Vector3d var = (sum_sq_ - sum_.cwiseProduct(sum_) / n) / (n - 1.0);
    return var.cwiseMax(0.0).cwiseSqrt();
  }

 private:
  Params p_;
  Eigen::Vector3d sum_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_sq_ = Eigen::Vector3d::Zero();
  std::size_t n_ = 0;
  bool done_ = false;
  Eigen::Vector3d delta_theta_ = Eigen::Vector3d::Zero();
  Sophus::SO3d R_wb_;
};

}  // namespace gravity
