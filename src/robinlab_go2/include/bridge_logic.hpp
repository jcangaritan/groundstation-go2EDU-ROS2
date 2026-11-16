// bridge_logic.hpp - Decide QUÉ mandar al robot a partir de cada cmd_vel. Sin ROS, sin Unitree.
//   cmd_vel -> sanea (NaN/inf = parada) -> limita -> {Stop | Move}; + watchdog.
//   El nodo solo traduce Action a llamadas SportClient::StopMove / Move.
#pragma once
#include <algorithm>
#include <cmath>
#include "command_watchdog.hpp"

namespace tele {

struct BridgeLimits {
  double max_vx = 3.0;
  double max_vy = 1.0;
  double max_wz = 1.0;
  int64_t timeout_ns = 500'000'000;
};

struct Action {
  enum Kind { None, Stop, Move } kind = None;
  double vx = 0.0, vy = 0.0, wz = 0.0;
};

class BridgeLogic {
 public:
  explicit BridgeLogic(BridgeLimits l = BridgeLimits()) : l_(l), wd_(l.timeout_ns) {}

  Action onCmd(double vx, double vy, double wz, int64_t t_ns) {
    Action a;
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {   // orden corrupta -> fail-safe
      ++rejected_;
      wd_.onCommand(false, t_ns);
      a.kind = Action::Stop;
      return a;
    }
    vx = std::clamp(vx, -l_.max_vx, l_.max_vx);
    vy = std::clamp(vy, -l_.max_vy, l_.max_vy);
    wz = std::clamp(wz, -l_.max_wz, l_.max_wz);
    if (vx == 0.0 && vy == 0.0 && wz == 0.0) {
      wd_.onCommand(false, t_ns);
      a.kind = Action::Stop;
    } else {
      wd_.onCommand(true, t_ns);
      a.kind = Action::Move; a.vx = vx; a.vy = vy; a.wz = wz;
    }
    return a;
  }

  Action onTick(int64_t t_ns) {
    Action a;
    if (wd_.poll(t_ns)) a.kind = Action::Stop;
    return a;
  }

  unsigned rejected() const { return rejected_; }

 private:
  BridgeLimits l_;
  CommandWatchdog wd_;
  unsigned rejected_ = 0;
};

}  // namespace tele

