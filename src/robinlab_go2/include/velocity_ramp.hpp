// velocity_ramp.hpp - Limitador de aceleración (rampa). Sin ROS. Una sola responsabilidad.
// Trabaja con velocidad >= 0 (crucero hacia delante). accel: subida, decel: bajada [m/s^2].
#pragma once
#include <algorithm>

namespace tele {

struct RampParams {
  double accel = 0.5;   // m/s^2 al acelerar
  double decel = 1.0;   // m/s^2 al frenar (más alto que accel: parar rápido es más seguro)
};

class VelocityRamp {
 public:
  explicit VelocityRamp(RampParams p = RampParams()) : p_(p) {}
  // Avanza dt segundos hacia 'target'. Nunca supera la pendiente permitida ni se pasa del objetivo.
  double step(double target, double dt) {
    if (dt <= 0.0) return v_;
    if (v_ < target)      v_ = std::min(target, v_ + p_.accel * dt);
    else if (v_ > target) v_ = std::max(target, v_ - p_.decel * dt);
    return v_;
  }
  double value() const { return v_; }
  void reset(double v = 0.0) { v_ = v; }
 private:
  RampParams p_;
  double v_ = 0.0;
};

}  // namespace tele

