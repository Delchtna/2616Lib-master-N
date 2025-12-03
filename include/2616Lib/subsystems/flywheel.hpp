#pragma once

#include "pros/motors.hpp"
#include "2616Lib/util/PIDF.hpp"
#include "2616Lib/util/points/pose.hpp"
#include "2616Lib/util/simple_moving_average.hpp"
#include <vector>

class Flywheel {
  public:
    Flywheel(std::vector<int> motor_ports, double ratio, pros::motor_gearset_e_t cartridge = pros::E_MOTOR_GEAR_600);
    
    void set_constants(double p, double i, double d, double p_start_i, double f, double pidf_threshold);
    void reset_pidf_variables();
    
    void handle_speed_controls();
    void set_target(double target);
    void flywheel_task_func();

    double get_ratio();
    void set_ratio(double ratio);

    std::vector<pros::Motor> get_motors();


  private:
    std::vector<pros::Motor> motors;
    pros::motor_gearset_e_t cartridge;
    double ratio;
    
    Simple_Moving_Average moving_avg;
    
    double target;
    double bb_threshold;

    PIDF pidf;
};