#include "2616Lib/util/util.hpp"
#include "main.h"


//Calculates robot's absolute position based on sensor values and trigonometry
//Based on: https://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf, https://www.youtube.com/watch?v=GEZBYHVHmFQ
void Chassis::odom_task_func() {
  //Check if each of the sensors is enabled
  bool is_perpendicular_enabled = perpendicular_tracker.is_enabled();
  bool is_left_enabled = left_tracker.is_enabled();
  bool is_right_enabled = right_tracker.is_enabled();
  
  //Get the offset for each tracking wheel (used for calculations)
  double perpendicular_offset = perpendicular_tracker.get_offset();
  double left_offset = left_tracker.get_offset();
  double right_offset = right_tracker.get_offset();

  std::pair<double, bool> perpendicular_tracker_status, left_tracker_status, right_tracker_status;
  bool is_perpendicular_error = false, is_left_error = false, is_right_error = false;

  double total_perpendicular_dist = 0, total_left_dist = 0, total_right_dist = 0;
  double delta_perpendicular = 0, delta_left = 0, delta_right = 0;
  double prev_perpendicular_dist = 0, prev_right_dist = 0, prev_left_dist = 0;
  
  double local_x = 0, local_y = 0;
  double delta_theta = 0, alpha = 0;
  //All rotation values are stored in radians, unless named with `_deg`
  double total_imu_rotation_deg = 0, total_imu_theta = 0, prev_imu_theta = 0;
  bool prev_is_imu_error = false;
  
  
  while (true) {
    odom_mutex.take();

    //Get the current distance traveled values from each of the odom sensors and store it in the `first` variable of the status.
    //The `second` variable represents is true if the tracking wheel throws an error while enabled, and is always false if the tracking wheel is disabled.
    if (is_perpendicular_enabled) {
      perpendicular_tracker_status = perpendicular_tracker.get_status();
      total_perpendicular_dist = perpendicular_tracker_status.first;
      is_perpendicular_error = perpendicular_tracker_status.second;
    }
    if (is_left_enabled) {
      left_tracker_status = left_tracker.get_status();
      total_left_dist = left_tracker_status.first;
      is_left_error = left_tracker_status.second;
    }
    if (is_right_enabled) {
      right_tracker_status = right_tracker.get_status();
      total_right_dist = right_tracker_status.first;
      is_right_error = right_tracker_status.second;
    }


    //Check if any of the sensors are throwing errors
    if (is_perpendicular_error || is_left_error || is_right_error) {
      //One of the sensors is throwing errors, so don't update the odom position during this loop to prevent unexpected behavior from doing math using error values
      update_odom.store(false);
      //Print which sensor is throwing an error
      if (is_perpendicular_error) {
        printf("Perpendicular tracker is throwing an error!\n");
      }
      if (is_left_error) {
        printf("Left tracker is throwing an error!\n");
      }
      if (is_right_error) {
        printf("Right tracker is throwing an error!\n");
      }

      std::cout << strerror(errno) << std::endl;
    } else {
      update_odom.store(true);
    }
    

    if (update_odom.load()) {
      //Get the change in each sensor value since the previous iteration
      delta_perpendicular = total_perpendicular_dist - prev_perpendicular_dist;
      delta_left = total_left_dist - prev_left_dist;
      delta_right = total_right_dist - prev_right_dist;
      

      //Get the angle of the robot by reading the IMUs or calculating it based on the left and right sensor values
      total_imu_rotation_deg = get_imu_rotation();
      if (!imu_sensors.empty() && !is_imu_error && !prefer_calculated_odom_angle) {
        //All IMUs are working, and IMUs are the preferred method for tracking the robot's angle
        total_imu_theta = Util::to_rad(total_imu_rotation_deg);
        
        if (prev_is_imu_error && !is_imu_error) {
          //IMUs were throwing errors on the previous iteration but are working now, so reset the previous IMU theta to match the current angle instead of the angle when the IMUs first started throwing errors
          prev_imu_theta = total_imu_theta;
        }
        
        delta_theta = total_imu_theta - prev_imu_theta;
        prev_imu_theta = total_imu_theta;
      } else {
        //IMUs are not connected, an IMU is throwing an error, or this is the preferred method for tracking the robot's angle, so calculate angle using tracking wheels if possible

        if (ODOM_TYPE == e_odom_type::THREE_WHEEL || ODOM_TYPE == e_odom_type::DOUBLE_PARALLEL_IMU) {
          //Odom type supports a wheel-calculated angle
          delta_theta = (delta_left - delta_right) / (left_offset + right_offset);
        } else {
          printf("There are no IMUs defined, or an IMU is throwing errors, and this odom type does not support a wheel-calculated odom angle! Disabling odometry...\n");
          update_odom.store(false);
          return;
        }
      }


      //Update the previous iteration's variables using the current ones (these new "previous" values won't be used again until the next iteration)
      prev_right_dist = total_right_dist;
      prev_left_dist = total_left_dist;
      prev_perpendicular_dist = total_perpendicular_dist;
      prev_is_imu_error = is_imu_error;


      //Calculate local x and y coordinates, where the origin is the previous robot position and the positive y-axis is the line between the previous and current robot positions
      //Math explanation: https://www.youtube.com/watch?v=qqODIdvSGac
      if (delta_theta != 0) {
        //Robot turned in any direction

        if (is_right_enabled && is_left_enabled) {
          //Both left and right trackers are enabled, so calculate the local y independently for both and then average them
          local_y = 
                    (((delta_right / delta_theta) + right_offset) * 2 * sin(delta_theta / 2) +
                    ((delta_left / delta_theta) - left_offset) * 2 * sin(delta_theta / 2)) / 2;
                    
        } else if (is_right_enabled) {
          //Right tracker is preferred for finding this value, so use the right tracker version of the calculation
          local_y = ((delta_right / delta_theta) + right_offset) * 2 * sin(delta_theta / 2);
        } else {
          //Right tracker is not enabled, so use the left tracker version of the calculation
          //There shouldn't ever be a situation where both the left and right trackers are disabled since odom would be disabled
          local_y = ((delta_left / delta_theta) - left_offset) * 2 * sin(delta_theta / 2);
        }

        if (is_perpendicular_enabled) {
          local_x = ((delta_perpendicular / delta_theta) + perpendicular_offset) * 2 * sin(delta_theta / 2);
        }

      } else {
        //Robot moved perfectly straight or didn't move (this section prevents divide by 0 errors)

        if (is_right_enabled) {
          local_y = delta_right;
        } else {
          //There shouldn't ever be a situation where both the left and right trackers are disabled since odom would be disabled
          local_y = delta_left;
        }
        if (is_perpendicular_enabled) {
          local_x = delta_perpendicular;
        }
      }


      //Rotate the local x and y coordinates onto the global coordinate plane using matrix multiplication, then use them to update the global x, y, and angle
      //A positive angle should mean clockwise rotation, so use the reversed version of the rotation matrix: https://en.wikipedia.org/wiki/Rotation_matrix#Direction
      alpha = robot_pose.angle + (delta_theta / 2);
      robot_pose.x += Util::x_rotate_point(local_x, local_y, alpha, false);
      robot_pose.y += Util::y_rotate_point(local_x, local_y, alpha, false);
      robot_pose.angle += delta_theta;

      
      //Normalize robot angle in [-360, 360]
      robot_pose.angle = fmod(robot_pose.angle, 2 * M_PI);
    }


    odom_mutex.give();
    pros::delay(Util::DELAY_TIME);
  }
}

//Set position of the robot on the field with independent x, y, and angle values
void Chassis::set_odom_position(double x, double y, double angle) {
  std::lock_guard<pros::Mutex> guard(odom_mutex);
  
  robot_pose.x = x;
  robot_pose.y = y;
  robot_pose.angle = Util::to_rad(angle);
}

//Set the x, y, and angle of the robot's odom
void Chassis::set_odom_position(Pose pose) {
  std::lock_guard<pros::Mutex> guard(mutex);
  
  robot_pose = Pose(pose.x, pose.y, Util::to_rad(pose.angle));
}

//Set the x position of the robot's odom (in degrees)
void Chassis::set_odom_x(double x) {
  std::lock_guard<pros::Mutex> guard(odom_mutex);
  
  robot_pose.x = x;
}

//Set the y position of the robot's odom
void Chassis::set_odom_y(double y) {
  std::lock_guard<pros::Mutex> guard(odom_mutex);
  
  robot_pose.y = y;
}

//Set the angle of the robot's odom (in degrees)
void Chassis::set_odom_angle(double degrees) {
  std::lock_guard<pros::Mutex> guard(odom_mutex);

  robot_pose.angle = Util::to_rad(degrees);
}

