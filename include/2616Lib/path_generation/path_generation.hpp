#pragma once

#include "2616Lib/path_generation/quintic_bezier.hpp"
#include "2616Lib/util/points/path_point.hpp"

namespace Path_Generation {
  std::vector<Quintic_Bezier> generate_path(std::vector<Point> points, double initial_angle, double tangent_magnitude);
  std::vector<Quintic_Bezier> generate_path(std::vector<Point> points, double initial_angle, double final_angle, double tangent_magnitude);
  std::vector<Quintic_Bezier> generate_path(std::vector<Pose> poses, double tangent_magnitude);
  std::vector<Path_Point> calc_velocity(std::vector<Path_Point> path, double v, double a);
  std::vector<Path_Point> calculate_trajectory(std::vector<Quintic_Bezier> path, double v0, double v1, double max_v, double a_accel, double a_decel, double max_w);
  
   std::vector<Path_Point> calculate_trajectory(std::vector<Quintic_Bezier> path, int num_points, double v0, double v1, double max_v, double a_accel, double a_decel, double max_w);
  
  
  Quintic_Bezier calc_bezier_curve(int curr, std::vector<Pose> path, double tangent_magnitude);
  Quintic_Bezier calc_bezier_curve(int curr, std::vector<Point> path, double initial_angle, double final_angle, double tangent_magnitude);
  Quintic_Bezier calc_bezier_curve(int curr, std::vector<Point> path, double initial_angle, double tangent_magnitude);

  double trapezoidal_motion_profile(double distance, double total_dist, double v0, double v1, double v_max, double a_accel, double a_decel);
}

