#include "2616Lib/util/util.hpp"
#include "main.h"
#include "pros/error.h"


// ************************ CONSTRUCTOR ************************

Chassis::Chassis(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, std::vector<int> imu_sensor_ports, 
                 std::vector<Tracking_Wheel> trackers, bool disable_odom):
    //Initialize the tracking wheels based on the type specified in each vector element. If disable_odom is true, set all tracking wheels to the empty constructor
    //This works based on the copy/move constructors, where each member tracking wheel is initialized as a copy of either an object in the vector, or an empty object if the type is not matched
    perpendicular_tracker(Tracking_Wheel::find_by_type(trackers, Tracking_Wheel::e_tracker_type::PERPENDICULAR, disable_odom)),
    //perpendicular_tracker(Tracking_Wheel::find_by_type(trackers, Tracking_Wheel::e_tracker_type::PERPENDICULAR, disable_odom)),
    left_tracker(Tracking_Wheel::find_by_type(trackers, Tracking_Wheel::e_tracker_type::LEFT, disable_odom)),
    right_tracker(Tracking_Wheel::find_by_type(trackers, Tracking_Wheel::e_tracker_type::RIGHT, disable_odom)),
    //Initialize ODOM_TYPE based on which combination of tracking wheels are enabled, or set to NONE if odom disabled
    ODOM_TYPE(disable_odom ? e_odom_type::NONE : determine_odom_type()) {


  //Fill motor and IMU vectors
  for (auto i : imu_sensor_ports) {
    pros::IMU temp(i);
    imu_sensors.push_back(temp);
  }
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), Util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), Util::is_reversed(i));
    right_motors.push_back(temp);
  }

  set_drive_mode(e_drive_mode::STANDBY);
  set_odom_position(0, 0, 0);
  target_pose = Pose(0, 0, 0);

  //TODO: vibrate controller if any of chassis motors are disabled - check if value of each == 0? check errno directly? make sure this doesn't cause issues with getting info from the electronics before they are initialized (which could crash the program)
}


//Determine the type of odometry used by this chassis based on which tracking wheels are enabled
Chassis::e_odom_type Chassis::determine_odom_type() {
  if (perpendicular_tracker.is_enabled()) {
    //Perpendicular tracker enabled
    if (left_tracker.is_enabled() && right_tracker.is_enabled()) {
      //All three trackers enabled
      return Chassis::e_odom_type::THREE_WHEEL;
    } else if (left_tracker.is_enabled() || right_tracker.is_enabled()) {
      //Perpendicular and left or right (but not both) trackers enabled
      return Chassis::e_odom_type::TWO_WHEEL;
    }
    //Else -> only perpendicular tracker is enabled, so no odom
  } else {
    //Perpendicular tracker disabled
    if (left_tracker.is_enabled() && right_tracker.is_enabled()) {
      //Left and right trackers enabled, perpendicular tracker disabled
      return Chassis::e_odom_type::DOUBLE_PARALLEL_IMU;
    } else if (left_tracker.is_enabled() || right_tracker.is_enabled()) {
      //Left or right (but not both) trackers enabled, perpendicular tracker disabled
      return Chassis::e_odom_type::SINGLE_PARALLEL_IMU;
    }
    //Else -> all trackers are disabled, so no odom
  }
  
  //None of the other odom patterns match, so disable odom
  return Chassis::e_odom_type::NONE;
}

//Start all chassis tasks
void Chassis::start_tasks() {
  printf("Starting chassis tasks with ODOM_TYPE = %i\n", ODOM_TYPE);

  if (ODOM_TYPE == e_odom_type::NONE) {
    update_odom.store(false);
    printf("Odometry is disabled, so the chassis tasks were not started! Odometry and movements are both disabled!\n");
  } else {    
    update_odom.store(true);

    reset_drive_sensors();
    imu_reset();

    pros::Task odom_task([this] { this->odom_task_func(); });
    pros::Task movement_task([this] { this->movement_task_func(); });

    pros::delay(100);
  }
  
  pros::Task brain_printing_task([this] { this->brain_printing_task_func(); });
}

//Get the current drive mode
Chassis::e_drive_mode Chassis::get_drive_mode() { 
  std::lock_guard<pros::Mutex> guard(mutex);
  return drive_mode; 
}

//Set the current drive mode
void Chassis::set_drive_mode(e_drive_mode new_mode) { 
  std::lock_guard<pros::Mutex> guard(mutex);
  drive_mode = new_mode; 
}

//Get the current robot pose
Pose& Chassis::get_pose() { 
  std::lock_guard<pros::Mutex> guard(mutex);
  return this->robot_pose;
}

//Set the brake mode of all drive motors
void Chassis::set_brake_mode(pros::motor_brake_mode_e_t brake_type) {
  for (auto motor : left_motors) {
    motor.set_brake_mode(brake_type);
  }
  for (auto motor : right_motors) {
    motor.set_brake_mode(brake_type);
  }
}

//Set active brake power and threshold
void Chassis::set_active_brake_power(double kP, int threshold) {
  active_brake_kp = fabs(kP);
  active_brake_threshold = abs(threshold);
}


//Set joystick curve scale and type.
//curve_type (must be either "red" or "blue") refers to this graph: https://www.desmos.com/calculator/rcfjjg83zx, where red is default
void Chassis::set_joystick_curve(double scale, std::string curve_type) {
  joystick_curve_scale = scale;

  //Make sure curve_type is either "red" or "blue" before saving
  if (curve_type.compare("red") == 0 || curve_type.compare("blue") == 0) {
    joystick_curve_type = curve_type;
  } else {
    printf("Tried to set the joystick curve type to a value other than \"red\" or \"blue\", so saving default value of \"red\" instead!\n");
    joystick_curve_type = "red";
  }
}

//Modify TANK input based on joystick curves, set drive speed, enable and run active brake if joysticks below threshold
void Chassis::tank_drive() {
  //Get input from the joysticks
  int left_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int right_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
  
  //Modify joystick input using drive curves
  int left_power = joystick_curve_function(left_stick);
  int right_power = joystick_curve_function(right_stick);

  //Both joysticks are less than the threshold, so enable active braking
  if (active_brake_kp != 0 && abs(left_stick) < active_brake_threshold && abs(right_stick) < active_brake_threshold) {
    //When joysticks are released, run active brake on drive using P loop
    //This works by getting the difference of the left and right tracker to their values before active brake was enabled, then multiplying by kp and -1 to swap the direction
    set_tank((left_motors.front().get_position() - left_brake_set_point) * active_brake_kp * -1,
            (right_motors.front().get_position() - right_brake_set_point) * active_brake_kp * -1);
  
  } else {
    //Joysticks are past the threshold, so set wheels to joystick power like normal
    set_tank(left_power, right_power);

    //Update the brake set point in ticks so it can be used if active brake turns on
    left_brake_set_point = left_motors.front().get_position();
    right_brake_set_point = right_motors.front().get_position();
  }
}


//Modify ARCADE input based on joystick curves, set drive speed, enable and run active brake if joysticks below threshold
void Chassis::arcade_drive(bool flipped) {
  //Get input from the joysticks
  int forward_stick, turn_stick;
  
  if (!flipped) {
    //Normal mode where LEFT_Y is forward and RIGHT_X is turn
    forward_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    turn_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  } else {
    //Flipped mode where RIGHT_Y is forward and LEFT_X is turn
    forward_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    turn_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
  }

  //Modify joystick input using drive curves
  int forward_power = joystick_curve_function(forward_stick);
  int turn_power = joystick_curve_function(turn_stick);
  
  
  //Both joysticks are less than the threshold, so enable active braking
  if (active_brake_kp != 0 && abs(forward_stick) < active_brake_threshold && abs(turn_stick) < active_brake_threshold) {
    //When joysticks are released, run active brake on drive using P loop
    //This works by getting the difference of the left and right tracker to their values before active brake was enabled, then multiplying by kp and -1 to swap the direction
    set_tank((left_motors.front().get_position() - left_brake_set_point) * active_brake_kp * -1,
            (right_motors.front().get_position() - right_brake_set_point) * active_brake_kp * -1);
  
  } else {
    //Joysticks are past the threshold, so set wheels to joystick power like normal
    set_tank(forward_power + turn_power, forward_power - turn_power);

    //Update the brake set point in ticks so it can be used if active brake turns on
    left_brake_set_point = left_motors.front().get_position();
    right_brake_set_point = right_motors.front().get_position();
  }
}

  //Test to determine relationship between voltage and velocity of the robot.
  void Chassis::ramp_voltage_test(double volts_per_second){
    double current_voltage = 0;
    const double increment = (volts_per_second * 127)/(12 * (1000.0/Util::DELAY_TIME));
    double left, right;

    while (current_voltage < 127){
      set_tank(current_voltage, current_voltage);

      left = ((chassis.left_motors[0].get_actual_velocity()  * path_traverser.ratio) * path_traverser.wheel_diameter * M_PI) / 60;
      right = ((chassis.right_motors[0].get_actual_velocity()  * path_traverser.ratio) * path_traverser.wheel_diameter * M_PI) / 60;

      std::cout << (127.0/12000) * ((chassis.left_motors[0].get_voltage() + chassis.right_motors[0].get_voltage())/2.0) << " " << (left + right)/2 << std::endl;

      current_voltage += increment;
      pros::delay(Util::DELAY_TIME);
    }
  }

//Test to determine the effective track width of the robot.
void Chassis::turn_test(double volts){
  const double adjusted_volts = volts * (127.0/12000);
  chassis.left_motors.at(0).tare_position();
  chassis.right_motors.at(0).tare_position();
  while (true){
    set_tank(adjusted_volts, 0);
    double left = (chassis.left_motors.at(0).get_position() * path_traverser.wheel_diameter * M_PI)/ (path_traverser.ratio * 400);
    double right = (chassis.right_motors.at(0).get_position() * path_traverser.wheel_diameter * M_PI)/ (path_traverser.ratio * 400);

    double d_theta = Util::to_rad(get_imu_rotation());
    double total_right_dist = perpendicular_tracker.get_status().first;

    std::cout << (left-right)/d_theta << " " << total_right_dist / d_theta<< std::endl;
    pros::delay(50);
  }
}

  //Test to determine relationship between voltage and velocity of the robot..
  void Chassis::constant_voltage_test(double volts){
    volts = volts * (127.0/12);
    double velocity;
    Point last_position(0, 0);
    Point current_position;
    while (true){
      set_tank(volts, volts);
      current_position = Point(chassis.get_pose().x, chassis.get_pose().y);
      velocity = Util::magnitude(current_position - last_position)/(20/1000.0);

      // left = ((chassis.left_motors[0].get_actual_velocity()  * path_traverser.ratio) * path_traverser.wheel_diameter * M_PI) / 60;
      // right = ((chassis.right_motors[0].get_actual_velocity()  * path_traverser.ratio) * path_traverser.wheel_diameter * M_PI) / 60;

      std::cout << (127.0/12000) * ((chassis.left_motors[0].get_voltage() + chassis.right_motors[0].get_voltage())/2.0) << " " << velocity << std::endl;
      last_position = current_position;


      pros::delay(20);
    }
  }

//Implementation of the 5225 curves from 2018: https://www.desmos.com/calculator/rcfjjg83zx.
//The x-axis is the joystick input and the y-axis is the motor output.
//With a joystick curve, more of the joystick movement goes to lower speeds, giving you more control of the robot.
double Chassis::joystick_curve_function(double input) {
  if (joystick_curve_scale == 0) {
    return input;
  }

  if (joystick_curve_type.compare("red") == 0) {
    //Red curve
    return (powf(2.718, -(joystick_curve_scale / 10)) + powf(2.718, (fabs(input) - 127) / 10) * (1 - powf(2.718, -(joystick_curve_scale / 10)))) * input;
  } else if (joystick_curve_type.compare("blue") == 0) {
    //Blue curve
    return powf(2.718, ((fabs(input) - 127) * joystick_curve_scale) / 1000) * input;
  } else {
    printf("Invalid joystick curve type!\n");
    return input;
  }
}


//Set the preferred method for tracking the robot's angle. True means odom angle should be tracked using the left and right tracking wheels (if possible), false means odom angle should be tracked using IMUs (if possible)
void Chassis::prefer_wheel_calculated_odom_angle(bool prefer_wheel_calculation) {
  //Only prefer a wheel-calculated odom angle if the parameter is true, and if the current odom type supports it
  //If the parameter is false (meaning IMU is preferred), the overall stored bool will be false
  if (prefer_wheel_calculation) {
    if (ODOM_TYPE == e_odom_type::THREE_WHEEL || ODOM_TYPE == e_odom_type::DOUBLE_PARALLEL_IMU) {
      //This odom type supports a calculated odom angle
      prefer_calculated_odom_angle = true;
    } else {
      //This odom type DOES NOT support a calculated odom angle
      prefer_calculated_odom_angle = false;
      printf("The odom angle is set to be calculated using the left and right tracking wheels, but the current odom configuration does not support this! So, the odom angle will be tracked using IMUs instead.\n");
    }
  }
}


//Return the average rotation value of all the IMUs in degrees, or PROS_ERR_F if one of the IMUs throws an error
double Chassis::get_imu_rotation() {
  if (imu_sensors.empty()) { return 0; }

  double total_rotation = 0;
  for (auto i : imu_sensors) {
    double rot = i.get_rotation();
    if (rot == PROS_ERR_F) {
      //One of the IMUs is throwing an error, so exit
      if (!is_imu_error) { //Only print this error message once per error instead of spamming it
        printf("At least one IMU is throwing an error while getting its rotation!\n");
      }
      
      is_imu_error = true;
      return 0;
    }

    total_rotation += rot;
  }

  //All IMUs are working correctly
  is_imu_error = false;
  return total_rotation / imu_sensors.size();
}

void Chassis::set_bound(double close_bound){
  this->close_bound = close_bound;
}

void Chassis::imu_reset() {
  for (auto i : imu_sensors) {
    i.reset(true);
  }
}

bool Chassis::imu_is_calibrating() {
  for (auto i : imu_sensors) {
    //If an IMU is throwing errors (e.g. when it's disconnected), `i.is_calibrating()` returns true, so only run that method if the IMUs are working correctly
    if (!is_imu_error && i.is_calibrating()) {
      return true;
    }
  }
  return false;
}

void Chassis::reset_drive_sensors() {
  std::lock_guard<pros::Mutex> guard(mutex);

  left_tracker.reset_position();
  right_tracker.reset_position();
  perpendicular_tracker.reset_position();
}

double Chassis::get_distance(Point target_pose) {
  return Util::distance_formula(get_pose(), target_pose);
}