#include "2616Lib/util/points/path_point.hpp"
#include "2616Lib/util/util.hpp"
#include "main.h"
#include "motion_profiling.hpp"
#include <cmath>
#include <sstream>

// Constructor for Motion_Profiling class, initializes the current point
Motion_Profiling::Motion_Profiling() {
  this->current_point = Path_Point();
}

// Sets the trajectory with a given vector of Path_Point objects
void Motion_Profiling::set_trajectory(std::vector<Path_Point>& trajectory) {
  this->trajectory = trajectory;
  current_index = 0;
  starting_position = (chassis.left_tracker.get_value_inches() + chassis.right_tracker.get_value_inches()) / 2;
  left_wheel_sma.set_period(sma_period);
  right_wheel_sma.set_period(sma_period);
  current_v = 0;

  at_end = false;
}


// Sets software-related constants for motion profiling
void Motion_Profiling::set_software_constants(double kP, double kV, double kA, double kS, double zeta, double beta,  double  minimum_time_to_update, double distance_threshold, double exit_distance, int sma_period) {
  this->kP = kP;
  this->kV = kV;
  this->kA = kA;
  this->kS = kS;
  this->zeta = zeta;
  this->beta = beta;
  this->sma_period = sma_period;
  this->minimum_time_to_update = minimum_time_to_update;
  this->distance_threshold = distance_threshold;
  this->exit_distance = exit_distance;
}

// Sets hardware-related constants for motion profiling
void Motion_Profiling::set_hardware_constants(double ratio, double track_width, double wheel_diameter) {
  this->ratio = ratio;
  this->track_width = track_width;
  this->wheel_diameter = wheel_diameter;
}

// Sets motion constraints for the profiling
void Motion_Profiling::set_constraints( double final_angle, double tangent_magnitude, double v1, double v_max, double a_accel, double a_decel, double w, bool reversed){
  this->final_angle = final_angle;
  this->tangent_magnitude = tangent_magnitude;
  this->v1 = v1;
  this->v_max = v_max;
  this->a_accel = a_accel;
  this->a_decel = a_decel;
  this->w = w;
  this->reversed = reversed;
}

// Prints the logs stored in log_data to the console
void Motion_Profiling::print_logs() {
    for (const auto& line : log_data) {
        std::cout << line << std::endl;
    }
}

// Updates the path with a vector of Point objects
void Motion_Profiling::update_path(std::vector<Point>& path){
  this->point_path = path;
  path_type = e_path_type::POINT;
}

// Updates the path with a vector of Pose objects
void Motion_Profiling::update_path(std::vector<Pose>& path){
  this->pose_path = path;
  path_type = e_path_type::POSE;
}

// Updates the trajectory based on the current path and constraints
void Motion_Profiling::update_trajectory() {
    std::vector<Path_Point> traj;  // Temporary storage for the new trajectory

    if (path_type == e_path_type::POINT) {
        // If the path type is POINT, create a trajectory based on points
        Point current_point(chassis.get_pose().x, chassis.get_pose().y);
        std::vector<Point> local_path = point_path;
        local_path.insert(local_path.begin(), current_point);  // Add the current point to the beginning of the path

        if (reversed) {
            // Generate trajectory for reversed motion
            traj = Path_Generation::calculate_trajectory(
                Path_Generation::generate_path(local_path, Util::to_deg(Util::normalize(chassis.get_pose().angle + M_PI)), final_angle + 180, tangent_magnitude),
                0, v1, v_max, a_accel, a_decel, w);
        } else {
            // Generate trajectory for forward motion
            traj = Path_Generation::calculate_trajectory(
                Path_Generation::generate_path(local_path, Util::to_deg(Util::normalize(chassis.get_pose().angle)), final_angle, tangent_magnitude),
                0, v1, v_max, a_accel, a_decel, w);
        }

    } else if (path_type == e_path_type::POSE) {
        // If the path type is POSE, create a trajectory based on poses
        Pose current_pose = chassis.get_pose();
        current_pose.angle = Util::to_deg(Util::normalize(current_pose.angle));
        std::vector<Pose> local_path = pose_path;
        local_path.insert(local_path.begin(), current_pose);  // Add the current pose to the beginning of the path

        if (reversed) {
            // Adjust angles for reversed motion
            for (Pose& p : local_path) {
                p.angle += 180;
            }
        }

        traj = Path_Generation::calculate_trajectory(Path_Generation::generate_path(local_path, tangent_magnitude), 0, v1, v_max, a_accel, a_decel, w);
    }

    if (reversed) {
        // Reverse the velocity and acceleration for the entire trajectory
        for (Path_Point& p : traj) {
            p.velocity *= -1;
            p.acceleration *= -1;
            p.pose.angle += M_PI;
        }
    }

    starting_position = (chassis.left_tracker.get_value_inches() + chassis.right_tracker.get_value_inches()) / 2;  // Recalculate the starting position
    set_trajectory(traj);  // Update the trajectory
    time_from_last_update = 0;  // Reset the time since the last update
}

// Updates the current point in the trajectory based on the index
void Motion_Profiling::update_current_point() {
  if (check_if_update()){
    update_trajectory();
  }

  current_point = trajectory.at(current_index); 
}

// Checks if the trajectory needs to be updated based on time and distance thresholds
bool Motion_Profiling::check_if_update(){
  if (minimum_time_to_update == 0){
    return false;
  }
  
  bool time_check = false;
  bool distance_check;
  if (path_type == e_path_type::POINT){
    distance_check = Util::magnitude(chassis.get_pose() - point_path.back()) > distance_threshold;
  }else{
    distance_check = Util::magnitude(chassis.get_pose() - pose_path.back()) > distance_threshold;

  }

  if (time_from_last_update > minimum_time_to_update){
    time_check = true;
    time_from_last_update = 0;
  }else{
    time_from_last_update += Util::DELAY_TIME;
  }

  return time_check && distance_check;
 
}

// Executes the Ramsete controller to adjust the robot's motion
void Motion_Profiling::run_ramsete_controller() {

  // Desired state values from the current trajectory point
  double desX = Util::inch_to_m(current_point.pose.x);
  double desY = Util::inch_to_m(current_point.pose.y);
  double des_theta = current_point.pose.angle;
  double des_vel = Util::inch_to_m(current_point.velocity);
  double des_angular_vel = current_point.angular_velocity;
   
   // Current state values from the chassis
  double x = Util::inch_to_m(chassis.get_pose().x);
  double y = Util::inch_to_m(chassis.get_pose().y);
  double theta = chassis.get_pose().angle;

  // Calculate the error terms
  double global_ex = desX - x;
  double global_ey = desY - y;

  // Convert the angles to the correct orientation
  theta = M_PI / 2 - theta;
  des_theta = M_PI / 2 - des_theta;

  //X and Y formulas are swapped intentionally
  double ex = cos(theta) * global_ex + sin(theta) * global_ey;
  double ey = -sin(theta) * global_ex + cos(theta) * global_ey;

  // Calculate the error in the angle
  double et = Util::find_min_angle(des_theta, theta);

  // Calculate the control constants
  double k = 2 * zeta * sqrt(des_angular_vel * des_angular_vel + beta * des_vel * des_vel);
  double vel = des_vel * cos(et) + k * ex; //ey in odoprom coords
  double angular_vel = des_angular_vel + k * et + beta * des_vel * sin(et) * ey / et;
  
  //Update the velocity and angular velocity based on the constraints
  vel = Util::m_to_inch(vel);

  //UNCOMMENT THIS TO ENABLE OVERSATURATION CONSTRAINTS.
  // double curvature = angular_vel / vel;
  // double max_reachable_velocity = (v_max * w) / (fabs(curvature) * v_max + w);   
  
  // if (vel > max_reachable_velocity){
  //   vel = max_reachable_velocity;
  //   angular_vel = vel * curvature;
  // }
  delta_v = vel - Util::m_to_inch(des_vel);
  delta_w = angular_vel - des_angular_vel;

  current_point.velocity = vel;
  current_point.angular_velocity = angular_vel;
}

void Motion_Profiling::calculate_wheel_speeds() {
  // Calculate the wheel speeds based on the current point
  double left_wheel_acceleration, right_wheel_acceleration;
  double current_left_velocity = ((chassis.left_motors[0].get_actual_velocity()  * ratio) * wheel_diameter * M_PI) / 60;
  double current_right_velocity = ((chassis.right_motors[0].get_actual_velocity()  * ratio) * wheel_diameter * M_PI) / 60;
  current_angle = chassis.get_pose().angle;
  double current_angular_velocity = (current_angle-last_angle) / (0.01);
  left_wheel_sma.add_data(current_left_velocity);
  right_wheel_sma.add_data(current_right_velocity);

  //Use SMA to filter data
  current_left_velocity = left_wheel_sma.get_mean();
  current_right_velocity = right_wheel_sma.get_mean();

  //Calculate the wheel velocities
  double angular_error = current_point.angular_velocity - current_angular_velocity;
  left_wheel_velocity = current_point.velocity - (current_point.angular_velocity * track_width / 2);
  right_wheel_velocity = current_point.velocity + (current_point.angular_velocity * track_width / 2);
  double current_velocity = (current_right_velocity + current_left_velocity) / 2;
  double calculated_velocity = (right_wheel_velocity + left_wheel_velocity) / 2;
  
  //Calculate the wheel accelerations
  if (prev_left_wheel_velocity == 0){
    left_wheel_acceleration = left_wheel_velocity/(Util::DELAY_TIME/1000.0);
  } else {
    left_wheel_acceleration = ((left_wheel_velocity * left_wheel_velocity - prev_left_wheel_velocity * prev_left_wheel_velocity) / (2 * prev_left_wheel_velocity * (Util::DELAY_TIME / 1000.0)));
  }
  if (prev_right_wheel_velocity == 0){
    right_wheel_acceleration = right_wheel_velocity/(Util::DELAY_TIME/1000.0);
  } else {
    right_wheel_acceleration = ((right_wheel_velocity * right_wheel_velocity - prev_right_wheel_velocity * prev_right_wheel_velocity) / (2 * prev_right_wheel_velocity * (Util::DELAY_TIME / 1000.0)));
  }

  //Calculate the wheel errors
  double left_error = left_wheel_velocity - current_left_velocity;
  double right_error = right_wheel_velocity - current_right_velocity;
  // double current_position = (chassis.left_tracker.get_value_inches() + chassis.right_tracker.get_value_inches()) / 2;
  // double delta_x = current_position - starting_position;
  // double error = current_point.s - delta_x;

  //Calculate the wheel outputs
  double left_output = left_wheel_velocity * kV + left_wheel_acceleration * kA + left_error * kP + (kS * Util::sgn(left_wheel_velocity)) ;
  double right_output = right_wheel_velocity * kV + right_wheel_acceleration * kA + right_error * kP + (kS * Util::sgn(left_wheel_velocity)) ;

  //Print debug data to stream. Change 1 to modify frequency of data output.
  if ((count % 1) == 0){
    std::cout << chassis.get_pose().x << " "
            << chassis.get_pose().y << " "
            << chassis.get_pose().angle << " "
            << current_point.pose.x << " "
            << current_point.pose.y << " "
            << current_point.pose.angle << " "
            << current_point.angular_velocity << " "
            << current_angular_velocity * -1 << " "
            << current_left_velocity << " "
            << left_wheel_velocity << " "
            << current_right_velocity << " "
            << right_wheel_velocity << " "
            << (current_left_velocity + current_right_velocity) / 2 << " "
            << current_point.velocity<<std::endl;
  }

  //Set the wheel outputs and update variables.
  prev_left_wheel_velocity = left_wheel_velocity;
  prev_right_wheel_velocity = right_wheel_velocity;
  last_angle = current_angle;

  chassis.set_tank(left_output, right_output);
  count += 1;
}

//Updates the motion profiling controller.
void Motion_Profiling::check_if_at_end() {
  if (current_index >= trajectory.size() - 1) {
    if (v1 == 0){
      chassis.set_tank(0, 0);
    }

    chassis.set_drive_mode(Chassis::e_drive_mode::STANDBY);
    at_end = true;
    this->point_path = {};
    this->pose_path = {};

    path_type = e_path_type::STANDBY;
    prev_left_wheel_velocity = 0;
    prev_right_wheel_velocity = 0;
    print_logs();
    
    last_angle=0;
    count = 0;
    right_wheel_sma.clear();
    left_wheel_sma.clear();
    this->current_point = Path_Point();
   } else {
    current_index += 1;
   }
}
