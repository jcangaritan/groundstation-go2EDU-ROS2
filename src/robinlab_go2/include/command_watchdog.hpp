// command_watchdog.hpp - Vigilante de "se acabaron las órdenes". Sin ROS.
//   Dispara UNA vez cuando el robot iba en movimiento y dejan de llegar órdenes durante timeout_ns.
//   Después no vuelve a disparar hasta que haya una nueva orden de movimiento
//   (el watchdog antiguo repetía StopMove cada 100 ms indefinidamente).
#pragma once
#include <cstdint>

namespace tele {

class CommandWatchdog {
 public:
  explicit CommandWatchdog(int64_t timeout_ns = 500'000'000) : timeout_ns_(timeout_ns) {}

  // Llamar en cada orden recibida. moving=false si la orden es de parada.
  void onCommand(bool moving, int64_t t_ns) { last_ns_ = t_ns; moving_ = moving; }

  // Llamar periódicamente. true = hay que parar ahora (una sola vez).
  bool poll(int64_t t_ns) {
    if (!moving_ || t_ns - last_ns_ < timeout_ns_) return false;
    moving_ = false;
    return true;
  }
  bool moving() const { return moving_; }

 private:
  int64_t timeout_ns_;
  int64_t last_ns_ = 0;
  bool moving_ = false;     // arranca en 'no moviéndose': sin StopMove espurio al arrancar el nodo
};

}  // namespace tele

