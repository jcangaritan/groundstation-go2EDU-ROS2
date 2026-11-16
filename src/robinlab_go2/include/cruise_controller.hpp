// cruise_controller.hpp - Máquina de estados del crucero. Sin ROS, sin reloj propio.
//   Start (botón A) -> velocidad objetivo = vel;  Stop (botón B) -> objetivo = 0.
//   Start es idempotente. Stop gana si llegan ambos en el mismo ciclo (lo decide quien llama).
//   Al terminar de frenar emite 'zero_tail' ceros exactos (para que el conversor reciba el 0 final)
//   y luego se calla (publish=false): no pisa a otros publicadores de cmd_vel.
#pragma once
#include "velocity_ramp.hpp"

namespace tele {

enum class CruiseEvent { Start, Stop };
enum class CruiseState { Idle, Cruising, Braking };

struct CruiseParams {
  double vel = 0.6;        // m/s de crucero
  RampParams ramp;         // accel / decel
  int zero_tail = 5;       // ceros finales publicados al parar
};

struct CruiseOutput {
  bool   publish = false;  // ¿hay que publicar cmd_vel este ciclo?
  double vx = 0.0;
};

class CruiseController {
 public:
  explicit CruiseController(CruiseParams p = CruiseParams()) : p_(p), ramp_(p.ramp) {}

  void handle(CruiseEvent e) {
    if (e == CruiseEvent::Start) { enabled_ = true; tail_ = 0; }
    else                         { enabled_ = false; }
  }

  CruiseOutput update(double dt) {
    const double v = ramp_.step(enabled_ ? p_.vel : 0.0, dt);
    CruiseOutput o;
    if (enabled_ || v > 0.0) {
      tail_ = p_.zero_tail;
      o.publish = true;
      o.vx = v;
    } else if (tail_ > 0) {
      --tail_;
      o.publish = true;
      o.vx = 0.0;            // cero EXACTO: el conversor lo traduce a StopMove
    }
    return o;
  }

  CruiseState state() const {
    if (enabled_) return CruiseState::Cruising;
    return ramp_.value() > 0.0 ? CruiseState::Braking : CruiseState::Idle;
  }
  bool enabled() const { return enabled_; }
  double velocity() const { return ramp_.value(); }

 private:
  CruiseParams p_;
  VelocityRamp ramp_;
  bool enabled_ = false;
  int tail_ = 0;
};

}  // namespace tele

