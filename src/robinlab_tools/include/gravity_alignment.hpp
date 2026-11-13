// gravity_alignment.hpp
//
// WHAT IT DOES
//   Given ONE accelerometer vector `a` (already a trusted mean, see accelerometer_sampler.hpp),
//   returns the rotation R_wb that puts the gravity direction on -z of the world frame.
//
// CONVENTION
//   a     : accelerometer reading at rest, in the sensor frame. It measures the reaction to
//           gravity, so it points UP (level sensor: a ~ +z).
//   R_wb  : rotation body -> world, world z up:  R_wb * a  is parallel to +z
//           (equivalently R_wb * g_body = [0, 0, -1]).
//   Yaw is NOT observable from gravity. R_wb is the minimal (geodesic) rotation, i.e. no twist
//   about the vertical.
//
// PROCESS
//   Step 1. Find delta_theta in the tangent space T_eps SO(3) ~ R^3
//             action 1: define the ideal direction (up = +z) and the measured direction (a/|a|)
//             action 2: rotation axis   n = (a x up) / |a x up|        (a x up = sin(theta) * n)
//             action 3: rotation angle  theta = atan2(|a x up|, a . up) (a . up = cos(theta))
//             action 4: compose the rotation vector  delta_theta = theta * n
//   Step 2. Perturb the identity
//             action 1: initial element  eps = I
//             action 2: apply the perturbation  R = eps (+) delta_theta = I * Exp(delta_theta)
//
//   Note: delta_theta is always perpendicular to up, so it lies in the horizontal plane, which is
//   the tangent plane of S^2 at the zenith. That is why only 2 degrees of freedom (roll, pitch)
//   are determined, and why delta_theta has no vertical (yaw) component.
//
// Pure C++ (Eigen + Sophus); no ROS.
#pragma once

#include <Eigen/Dense>
#include <sophus/so3.hpp>

#include <cmath>

namespace gravity {

// ---- Step 1 ---------------------------------------------------------------------------------
// delta_theta in so(3) ~ R^3: the rotation vector that takes a/|a| to +z.
inline Eigen::Vector3d gravityIncrement(const Eigen::Vector3d& a_in) {
  // action 1: ideal direction and measured direction (unit vectors, so units do not matter)
  const Eigen::Vector3d a = a_in.normalized();
  const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();

  // action 2: rotation axis (not yet normalised): a x up = sin(theta) * n
  const Eigen::Vector3d axis = a.cross(up);
  const double s = axis.norm();   // sin(theta)
  const double c = a.dot(up);     // cos(theta)

  // degenerate cases: a parallel or antiparallel to up (no unique axis)
  if (s < 1e-12) {
    if (c > 0.0) return Eigen::Vector3d::Zero();  // already aligned
    return M_PI * a.unitOrthogonal();             // upside down: 180 deg about any perpendicular axis
  }

  // action 3: angle. atan2 is stable near 0 and pi (acos is not).
  const double theta = std::atan2(s, c);

  // action 4: compose delta_theta = theta * n
  return axis / s * theta;
}

// ---- Step 2 ---------------------------------------------------------------------------------
// R = eps (+) delta_theta = I * Exp(delta_theta). With eps = I, left and right plus coincide.
inline Sophus::SO3d perturbIdentity(const Eigen::Vector3d& delta_theta) {
  const Sophus::SO3d eps;  // action 1: initial element, the identity
  return eps * Sophus::SO3d::exp(delta_theta);  // action 2: apply the perturbation
}

// Both steps together: accelerometer vector -> R_wb.
inline Sophus::SO3d alignmentFromAccel(const Eigen::Vector3d& a) {
  return perturbIdentity(gravityIncrement(a));
}

// ---- Cross-check ----------------------------------------------------------------------------
// Alternative with ZYX Euler angles and yaw = 0: R = Ry(pitch) * Rx(roll). Same tilt as
// alignmentFromAccel(); they differ only by a small yaw (~ roll*pitch/2).
inline Sophus::SO3d rollPitchFromAccel(const Eigen::Vector3d& a) {
  const double roll = std::atan2(a.y(), a.z());
  const double pitch = std::atan2(-a.x(), std::hypot(a.y(), a.z()));
  return Sophus::SO3d::exp(Eigen::Vector3d(0.0, pitch, 0.0)) *
         Sophus::SO3d::exp(Eigen::Vector3d(roll, 0.0, 0.0));
}

}  // namespace gravity
