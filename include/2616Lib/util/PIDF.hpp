#pragma once

#include "2616Lib/util/PID.hpp"

class PIDF: public PID {
  public:
    PIDF();
    PIDF(double p, double i, double d, double p_start_i, double f);

    void set_constants(double p, double i, double d, double p_start_i, double f);

    double get_kf();
    void set_kf(double f);

    //Computes the motor power
    double compute(double current);
  
  private:
    double kf;
};