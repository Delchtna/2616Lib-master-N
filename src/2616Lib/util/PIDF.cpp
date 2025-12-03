#include "main.h"

PIDF::PIDF() {
  reset_variables();
  set_constants(0, 0, 0, 0, 0);
}

//
PIDF::PIDF(double p, double i, double d, double p_start_i, double f) {
  reset_variables();
  set_constants(p, i, d, p_start_i, f);
}

void PIDF::set_constants(double p, double i, double d, double p_start_i, double f) {
  PID::set_constants(p, i, d, p_start_i);
  set_kf(f);
}

double PIDF::get_kf() { return kf; }
void PIDF::set_kf(double f) { kf = f; }

//Compute output of PIDF controller
double PIDF::compute(double current) {
    return (PID::compute(current) + (PID::get_target() * kf));
}