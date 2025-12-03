#include "main.h"
#include <cmath>


//Move the left side and right side drive motors at specific speeds within [-127, 127]
void Chassis::set_tank(double left, double right) {
 // std::lock_guard<pros::Mutex> guard(mutex);
  if (pros::millis() < 1500) { return; }

  for (auto i : left_motors) {
    i.move_voltage(left * (12000.0 / 127.0)); 
  }
  for (auto i : right_motors) {
     i.move_voltage(right * (12000.0 / 127.0));  
  }
}



// ************************ DRIVE METHODS ************************

void Chassis::drive(double distance, float speed, float min_speed) {
  reversed = Util::sgn(distance);
  target_pose =  Pose(sin(get_pose().angle) * distance + get_pose().x, cos(get_pose().angle) * distance + get_pose().y, Util::to_deg(get_pose().angle));
  
  double x_error = target_pose.x - get_pose().x;
  double y_error = target_pose.y - get_pose().y;

  this->starting_x_error_sgn = Util::sgn(x_error);
  this->starting_y_error_sgn = Util::sgn(y_error);

  double target_angle_rad = atan2(x_error, y_error);

  target_pose = Pose(sin(get_pose().angle) * distance + get_pose().x, cos(get_pose().angle) * distance + get_pose().y, Util::to_deg(get_pose().angle));

  drive_PID.set_target(distance);
  heading_PID.set_target(target_angle_rad);
  drive_PID.set_max_speed(speed);
  drive_PID.set_min_speed(min_speed);

  set_drive_mode(e_drive_mode::DRIVE);
}

void Chassis::drive(Pose pose, float speed, bool drive_reversed, float min_speed) {
  if (drive_reversed){
    reversed = -1;
  }else{
    reversed = 1;
  }
  
  double x_error = pose.x - get_pose().x;
  double y_error = pose.y - get_pose().y;

  this->starting_x_error_sgn = Util::sgn(x_error);
  this->starting_y_error_sgn = Util::sgn(y_error);

  double distance = std::hypot(x_error, y_error);
  double target_angle_rad = atan2(x_error, y_error);

  target_pose = pose;

  drive_PID.set_target(distance);
  heading_PID.set_target(target_angle_rad);
  drive_PID.set_max_speed(speed);
  drive_PID.set_min_speed(min_speed);

  set_drive_mode(e_drive_mode::DRIVE);
}

void Chassis::drive(Point point, float speed, bool drive_reversed, float min_speed) {
  if (drive_reversed){
    reversed = -1;
  }else{
    reversed = 1;
  }

  double x_error = point.x - get_pose().x;
  double y_error = point.y - get_pose().y;

  this->starting_x_error_sgn = Util::sgn(x_error);
  this->starting_y_error_sgn = Util::sgn(y_error);

  double distance = std::hypot(x_error, y_error);
  double target_angle_rad = atan2(x_error, y_error);
  
  target_pose = Pose(point.x, point.y, Util::to_deg(target_angle_rad));
  drive_PID.set_target(distance);
  heading_PID.set_target(target_angle_rad);
  drive_PID.set_max_speed(speed);
  drive_PID.set_min_speed(min_speed);

  set_drive_mode(e_drive_mode::DRIVE);
}



// ************************ TURN METHODS ************************

void Chassis::turn(double target, float speed) {
  target_angle = Util::to_rad(target);

  double target_angle_rad = Util::find_min_angle(target_angle, get_pose().angle);

  turn_PID.set_target(target_angle_rad);
  turn_PID.compute(0);
  turn_PID.set_max_speed(speed);

  set_drive_mode(e_drive_mode::TURN);
}


void Chassis::turn(Point point, float speed, bool away) {  
  turn(Util::get_angle_to_point(point, away), speed);
}

void Chassis::turn(Pose pose, float speed, bool away) {  
  turn(Util::get_angle_to_point(pose, away), speed);
}



// ************************ MISC MOVEMENTS ************************

void Chassis::arc(double target, e_arc_direction direction, float speed) {
  current_arc_direction = direction; 
  target_angle = Util::to_rad(target);

  double target_angle_rad = Util::find_min_angle(target_angle, get_pose().angle);

  arc_PID.set_target(target_angle_rad);
  arc_PID.compute(0);
  arc_PID.set_max_speed(speed);

  set_drive_mode(e_drive_mode::ARC);
}


void Chassis::motion_profiling(std::vector<Point> path, double final_angle, double tangent_magnitude, double v1, double v_max, double a_accel, double a_decel, double w, bool reversed) {

  path_traverser.set_constraints(final_angle, tangent_magnitude, v1, v_max, a_accel, a_decel,  w, reversed);
  path_traverser.update_path(path);
  long start = pros::millis();
  path_traverser.update_trajectory();

  std::cout <<  pros::millis() - start <<std::endl;
  path_traverser.prev_left_wheel_velocity = 0;
  path_traverser.prev_right_wheel_velocity = 0;
  set_drive_mode(e_drive_mode::MOTION_PROFILING);
}

void Chassis::motion_profiling(std::vector<Pose> path, double tangent_magnitude, double v1, double v_max, double a_accel, double a_decel, double w, bool reversed) {
  path_traverser.set_constraints(path.back().angle, tangent_magnitude, v1, v_max, a_accel, a_decel,  w, reversed);
  path_traverser.update_path(path);
  long start = pros::millis();
  path_traverser.update_trajectory();

  std::cout <<  pros::millis() - start <<std::endl;
  path_traverser.prev_left_wheel_velocity = 0;
  path_traverser.prev_right_wheel_velocity = 0;

  set_drive_mode(e_drive_mode::MOTION_PROFILING);
}

//Move to a point on the field with desired speed
void Chassis::move_to_point(Point point, float speed, float min_speed) {
  //Calculate target angle and turn towards target point    
  turn(point, speed);
  wait_drive();

  //Calculate target distance and drive towards point with heading correction
  drive(get_distance(point), speed, min_speed);
  wait_drive();
}

void Chassis::move_to_pose(Pose pose, float speed, bool turn_to_final_angle, float min_speed) {
  move_to_point(pose, speed, min_speed); //Execute the movement, excluding the final turn
  
  //Turn to the final angle
  if (turn_to_final_angle) {
    turn(pose.angle, speed);
    wait_drive();
  }
}

void Chassis::move_to_point(double xcoord, double ycoord, float speed,  float min_speed) {
  move_to_point(Point(xcoord, ycoord), speed, min_speed); 
}





// ************************ OTHER METHODS ************************

//Waits until the drive has finished a movement using exit conditions
void Chassis::wait_drive() {
  //Minimum movement time
  pros::delay(300); 
  
  if (drive_mode == e_drive_mode::DRIVE) {
    while (!drive_PID.is_settled()) {
      pros::delay(Util::DELAY_TIME);
    }
    set_drive_mode(e_drive_mode::STANDBY);
    
    //Reset drive and heading PID variables
    drive_PID.reset_variables();
    heading_PID.reset_variables();

  } else if (drive_mode == e_drive_mode::TURN) {
    while (!turn_PID.is_settled()) {
      pros::delay(Util::DELAY_TIME);
    }
    set_drive_mode(e_drive_mode::STANDBY);
    
    //Reset turn PID variables
    turn_PID.reset_variables();
      
 } else if (drive_mode == e_drive_mode::ARC) {
    while (!arc_PID.is_settled()) {
      pros::delay(Util::DELAY_TIME);
    }
    set_drive_mode(e_drive_mode::STANDBY);
    
    //Reset arc PID variables
    arc_PID.reset_variables();
      
  } else if (drive_mode == e_drive_mode::MOTION_PROFILING) {
    while (!path_traverser.at_end) {
      pros::delay(Util::DELAY_TIME);
    }
    set_drive_mode(e_drive_mode::STANDBY);
  }

  set_tank(0, 0);
}

