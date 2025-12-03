#include "2616Lib/chassis/odometry/tracking_wheel.hpp"
#include "main.h"


Tracking_Wheel::Tracking_Wheel() { disabled = true; }

//Integrated motor encoder - 1800 ticks/rev for red cart, 900 ticks/rev for green cart, 300 ticks/rev for blue cart
Tracking_Wheel::Tracking_Wheel(e_tracker_type type, pros::Motor motor, double wheel_diameter, double offset, double ratio) {
  get_value_func = std::bind(&pros::Motor::get_position, motor);
  reset_func = std::bind(&pros::Motor::tare_position, motor);

  double ticks_per_rev;
  switch (motor.get_gearing()) {
    case pros::E_MOTOR_GEAR_RED:
      ticks_per_rev = 1800;
      break;
    case pros::E_MOTOR_GEAR_GREEN:
      ticks_per_rev = 900;
      break;
    case pros::E_MOTOR_GEAR_BLUE:
      ticks_per_rev = 300;
      break;
    case pros::E_MOTOR_GEARSET_INVALID:
      printf("A tracking wheel was created using a motor with an invalid gearset, so it is being disabled!\n");
      disabled = true;
      break;
  }
  set_constants(type, wheel_diameter, ticks_per_rev, offset, ratio, PROS_ERR_F);
}

//Rotation sensor - 36000 ticks/rev
//Tracking_Wheel::Tracking_Wheel(e_tracker_type type, int rotation_port, double wheel_diameter, double offset, double ratio) {
Tracking_Wheel::Tracking_Wheel(e_tracker_type type, pros::Rotation rotation, bool reversed, double wheel_diameter, double offset, double ratio) {
  rotation.set_reversed(reversed); //TODO: remove this line and the `reversed` param and replace with `reversed` param inside sensor constructor - figure out why the program crashes otherwise - might have something to do with sensor initialization

  get_value_func = std::bind(&pros::Rotation::get_position, rotation);
  reset_func = std::bind(&pros::Rotation::reset_position, rotation);
  set_constants(type, wheel_diameter, 36000, offset, ratio, PROS_ERR);
}

//ADI encoder - 360 ticks/rev
Tracking_Wheel::Tracking_Wheel(e_tracker_type type, pros::ADIEncoder encoder, double wheel_diameter, double offset, double ratio) {
  get_value_func = std::bind(&pros::ADIEncoder::get_value, encoder);
  reset_func = std::bind(&pros::ADIEncoder::reset, encoder);
  set_constants(type, wheel_diameter, 360, offset, ratio, PROS_ERR);
}

bool Tracking_Wheel::is_enabled() { return !disabled; }

double Tracking_Wheel::get_offset() { return offset; }

Tracking_Wheel::e_tracker_type Tracking_Wheel::get_type() { return tracker_type; }

//Get the value of the sensor in ticks. If disabled, always returns 0
double Tracking_Wheel::get_value_ticks() {
  if (pros::millis() < 500) { return 0; }
  if (disabled) {
    printf("Tried to get the value of a disabled tracking wheel sensor, so returning 0!\n");
    return 0;
  }

  return get_value_func();
}

//Get the value of the sensor in inches using wheel size, ratio, and ticks per rotation
double Tracking_Wheel::get_value_inches() { 
  return convert_ticks_to_inches(get_value_ticks());
}

//Get the value of the sensor in inches based on the wheel size, ratio, and sensor type.
//The `double` element of the vector is the value. The `bool` element is true if the tracking wheel throws an error while enabled, and is always false if the tracking wheel is disabled.
std::pair<double, bool> Tracking_Wheel::get_status() { 
  double ticks = get_value_ticks();
  return { convert_ticks_to_inches(ticks), is_enabled() && fabs(ticks) == error_value };
}

double Tracking_Wheel::convert_ticks_to_inches(double ticks) {
  return (ticks * wheel_size * M_PI * ratio) / ticks_per_rev;
}

void Tracking_Wheel::reset_position() { if (is_enabled()) { reset_func(); } }

void Tracking_Wheel::set_constants(e_tracker_type type, double wheel_size, double ticks_per_rev, double offset, double ratio, double error_value) {
  this->tracker_type = type;
  this->wheel_size = wheel_size;
  this->ticks_per_rev = ticks_per_rev;
  this->offset = offset;
  this->ratio = ratio;
  this->error_value = error_value;
}

//Static method to check the vector for trackers that match a specific location value. If there is a match, return the tracker object. If not, return an empty tracker object.
Tracking_Wheel Tracking_Wheel::find_by_type(std::vector<Tracking_Wheel> trackers, Tracking_Wheel::e_tracker_type type, bool disable_odom) {
  //Return an iterator for the first tracker in the list that matches the location param
  auto it = std::find_if(trackers.begin(), trackers.end(), [type](Tracking_Wheel wheel) { return wheel.get_type() == type; });

  if (it != trackers.end() && !disable_odom) {
    //Found the desired tracker, so return it
    return *it;
  } else {
    //Did not find the desired tracker or odom is disabled, so return an empty tracker
    return Tracking_Wheel();
  }
}