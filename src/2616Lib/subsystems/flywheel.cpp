#include "main.h"
#include "pros/motors.h"

//Create a flywheel controller object using the port numbers of the motors. "ratio" must be a decimal!
//E.g. if you use 12:60 where the 12t is powered, your RATIO would be 0.2.
//E.g. if you use 12:84 where the 12t is powered, your RATIO would be 0.143.
Flywheel::Flywheel(std::vector<int> motor_ports, double ratio, pros::motor_gearset_e_t gearset):
    pidf(), ratio(ratio), cartridge(gearset) {
  
  //Fill the motor vector
  for (auto i : motor_ports) {
    pros::Motor temp(abs(i), gearset, Util::is_reversed(i));
    temp.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    motors.push_back(temp);
  }

  moving_avg.set_period(10);

  //Start main flywheel task
  pros::Task flywheel_task([this] { this->flywheel_task_func(); });
}

void Flywheel::set_constants(double p, double i, double d, double p_start_i, double f, double threshold) {
  pidf.set_constants(p, i, d, p_start_i, f); 
  this->bb_threshold = threshold;
}

void Flywheel::reset_pidf_variables() { pidf.reset_variables(); }

//Set the target flywheel RPM
void Flywheel::set_target(double target) { pidf.set_target(target); }

double Flywheel::get_ratio() { return ratio; }
void Flywheel::set_ratio(double ratio) { this->ratio = ratio; }

std::vector<pros::Motor> Flywheel::get_motors() { return motors; }

int flywheel_speed = 1800;
//Detect controller button presses and changes flywheel speed accordingly
void Flywheel::handle_speed_controls() {
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
    flywheel_speed = 0;
  }
  else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    flywheel_speed = 1800;
  }
  else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    flywheel_speed = 2400;
  }
  else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    flywheel_speed = 3000;
  }
  else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
  {
    flywheel_speed = 0;
  }
  
  set_target(flywheel_speed);
}

void Flywheel::flywheel_task_func() {
  for (auto i : motors) {
    i.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  }

  while (true) {
    double motor_power;
    double current = motors.front().get_actual_velocity() / ratio; 

    moving_avg.add_data(current);
    current = moving_avg.get_mean();

    double error = pidf.get_target() - current;

    if (error > bb_threshold) {
      motor_power = 12000;
    } else if (error < -bb_threshold) {
      motor_power = 0;
    } else {
      motor_power = pidf.compute(current);
    }

    //Make sure the motors never spin backward
    if (motor_power <= 0) { motor_power = 0; }

    for (auto i : motors) {
      i.move_voltage(motor_power);
    }

    pros::delay(Util::DELAY_TIME);
  }
}
