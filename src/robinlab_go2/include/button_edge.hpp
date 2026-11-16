// button_edge.hpp - "una pulsación = un evento", sin ROS (se prueba con g++ solo).
//
// Problema que resuelve
//   Leer el botón con un timer lento (200 ms) mira el ESTADO, no el CAMBIO:
//     - tap corto (<200 ms) que cae entre dos ticks      -> no se ve  (pulsación perdida)
//     - botón mantenido más de un tick                   -> se ve varias veces (doble lectura)
//     - rebote / ruido del mando                         -> se ve varias veces
//
// Solución: máquina de 2 estados por botón, alimentada a la velocidad del mensaje.
//
//        pulsado y ARMADO y fuera de lockout
//   ARMADO ───────────────────────────────────────► BLOQUEADO   (aquí se emite UN evento)
//      ▲                                                │
//      └──── suelto de forma estable >= release_ns ─────┘
//            y han pasado >= lockout_ns desde el evento
//
//   - El evento sale en el PRIMER mensaje con el botón pulsado (latencia ~0, no se pierde un tap).
//   - Mantener pulsado no genera más eventos: hay que soltar primero.
//   - Rebotes: si suelta y vuelve a pulsar antes de release_ns o de lockout_ns, se ignora.
//   - Limitación asumida: dos pulsaciones humanas separadas < lockout_ns cuentan como una.
#pragma once
#include <cstdint>

namespace btn {

struct EdgeParams {
  int64_t release_ns = 40'000'000;   // 40 ms: tiempo que debe estar SUELTO para rearmar
  int64_t lockout_ns = 200'000'000;  // 200 ms: mínimo entre dos eventos del mismo botón
};

class EdgeDetector {
 public:
  explicit EdgeDetector(EdgeParams p = EdgeParams()) : p_(p) {}

  // raw: botón pulsado en ESTE instante. t_ns: reloj monótono. Devuelve true UNA vez por pulsación.
  bool update(bool raw, int64_t t_ns) {
    if (armed_) {
      if (raw && (!has_event_ || t_ns - last_event_ >= p_.lockout_ns)) {
        armed_ = false;
        has_event_ = true;
        last_event_ = t_ns;
        releasing_ = false;
        return true;
      }
      return false;
    }
    // BLOQUEADO: esperar a que quede suelto de forma estable
    if (raw) {
      releasing_ = false;  // sigue (o volvió a estar) pulsado: reinicia la cuenta de suelto
    } else {
      if (!releasing_) { releasing_ = true; release_since_ = t_ns; }
      if (t_ns - release_since_ >= p_.release_ns && t_ns - last_event_ >= p_.lockout_ns) {
        armed_ = true;
        releasing_ = false;
      }
    }
    return false;
  }

  bool armed() const { return armed_; }
  void reset() { armed_ = true; has_event_ = false; releasing_ = false; }

 private:
  EdgeParams p_;
  bool armed_ = true;
  bool has_event_ = false;
  bool releasing_ = false;
  int64_t last_event_ = 0;
  int64_t release_since_ = 0;
};

}  // namespace btn

