#include "main.h"
#include <ostream>

//Handle PID and motion profiling movements asynchronously
void Chassis::movement_task_func() {
  while (true) {
    if (drive_mode == e_drive_mode::STANDBY) {
      //Do nothing
    } else if (drive_mode == e_drive_mode::TURN) {
      turn_pid_task();
    } else if (drive_mode == e_drive_mode::DRIVE) {
      drive_pid_task();
    } else if (drive_mode == e_drive_mode::ARC) {
      arc_pid_task();
    } else if (drive_mode == e_drive_mode::MOTION_PROFILING) {
      motion_profiling_task();
    }
    pros::delay(Util::DELAY_TIME);
  }
}

//Compute and normalize output of turn PID
void Chassis::turn_pid_task() {
  double target_angle_rad =  Util::find_min_angle(target_angle, get_pose().angle);
  turn_PID.set_target(target_angle_rad);
  turn_PID.compute(0);


  double out = Util::clip(turn_PID.get_output(), turn_PID.get_max_speed(), -turn_PID.get_max_speed());
  std::cout << target_angle_rad << " " << 0 << " " << out << std::endl;
  set_tank(out, -out);
}

//Compute and normalize output of drive PID
void Chassis::drive_pid_task() {
  int sgn = 1;
  double l_output, r_output;

  //Calculate target distances and angle of movement
  double distance = std::hypot(get_pose().x - target_pose.x, get_pose().y - target_pose.y);
  double x_error = target_pose.x - get_pose().x;
  double y_error = target_pose.y - get_pose().y;
  double target_angle_rad = atan2(x_error, y_error);

  bool x_sgn_not_equal = Util::sgn(x_error) != starting_x_error_sgn;
  bool y_sgn_not_equal = Util::sgn(y_error) != starting_y_error_sgn;

  //Reverse sgn based on change in x/y error, otherwise sgn stays equal to 1
  if ((x_sgn_not_equal && y_sgn_not_equal) ||
      (starting_x_error_sgn == 0 && y_sgn_not_equal) ||
      (x_sgn_not_equal && starting_y_error_sgn == 0)) {
    sgn = -1;
  }

  //If sgn is reversed, rotate target angle by 180 deg
  if ((reversed * sgn) == -1) {
    if (target_angle_rad <= 0) {
      target_angle_rad += M_PI;
    } else { //target_angle_rad > 0
      target_angle_rad -= M_PI;
    }
  }

  //Set PID target 

  drive_PID.set_target(distance * sgn * reversed);
  drive_PID.compute(0);
  double forward_voltage = drive_PID.get_output();

  //If robot is within some range of taget, turn to target angle.
  if (distance < close_bound) {
    double adjusted_angle = Util::find_min_angle(Util::to_rad(target_pose.angle), get_pose().angle);
    heading_PID.set_target(adjusted_angle);
    heading_PID.compute(0);

    forward_voltage *= std::cos(target_angle_rad - get_pose().angle);
    double turn_voltage =  heading_PID.get_output();
    
    l_output = forward_voltage + turn_voltage;
    r_output = forward_voltage - turn_voltage;

  } else {
    target_angle_rad =  Util::find_min_angle(target_angle_rad, get_pose().angle);
    heading_PID.set_target(target_angle_rad);
    heading_PID.compute(0);

    double turn_voltage =  heading_PID.get_output();
    l_output = forward_voltage + (turn_voltage * forward_voltage / 127) * Util::sgn(forward_voltage);
    r_output = forward_voltage - (turn_voltage * forward_voltage / 127) * Util::sgn(forward_voltage);
  }

  //Prevent motor oversaturation 
  if (fabs(l_output) > drive_PID.get_max_speed() || fabs(r_output) > drive_PID.get_max_speed()) {
    if (fabs(l_output) > fabs(r_output)) {
      double scale = drive_PID.get_max_speed() / fabs(l_output);
      r_output = r_output * scale;
      l_output = Util::clip(l_output, drive_PID.get_max_speed(), -drive_PID.get_max_speed());
    } else {
      double scale = drive_PID.get_max_speed() / fabs(r_output);
      l_output = l_output * scale;
      r_output = Util::clip(r_output, drive_PID.get_max_speed(), -drive_PID.get_max_speed());
    }
  }

    if (fabs(l_output) < drive_PID.get_min_speed()) {
      l_output = std::min(fabs(l_output), (double) drive_PID.get_min_speed()) * Util::sgn(l_output);
    }

    if ( fabs(r_output) < drive_PID.get_min_speed()){
      r_output = std::min(fabs(r_output), (double) drive_PID.get_min_speed()) * Util::sgn(r_output);
    }

  set_tank(l_output, r_output);
}

void Chassis::arc_pid_task() {
  double target_angle_rad =  Util::find_min_angle(target_angle, get_pose().angle);
  arc_PID.set_target(target_angle_rad);
  arc_PID.compute(0);
  double out = Util::clip(arc_PID.get_output(), arc_PID.get_max_speed(), -arc_PID.get_max_speed());

  if (current_arc_direction == e_arc_direction::RIGHT) {
    set_tank(0, -out);
  } else if (current_arc_direction == e_arc_direction::LEFT) {
    set_tank(out, 0);
  } 
}

void Chassis::motion_profiling_task() {
  path_traverser.update_current_point();
  if (update_odom.load()) {
    path_traverser.run_ramsete_controller();
  }
  path_traverser.calculate_wheel_speeds();
  path_traverser.check_if_at_end();

}
