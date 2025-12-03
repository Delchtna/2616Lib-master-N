#pragma once

#include "api.h"
#include "2616Lib/util/points/point.hpp"

namespace Util {
  //Delay time for tasks (measured in milliseconds)
  const int DELAY_TIME = 10;
  
  //Returns 1 if input is positive and -1 if input is negative
  int sgn(double input);
  //Returns if the input is less than 0
  bool is_reversed(double input);

  //Clip a number between [min, max]
  double clip(double d, double max, double min);

  double normalize(double angle);

  //Find minimum angle between two headings
  double find_min_angle(double target_heading, double current_heading);

  //Get the angle turn toward (or away from) a specific point
  double get_angle_to_point(Point target, bool away);


  double wrap(double num, double range);

  //Unit conversions
  double to_rad(double degrees);
  double to_deg(double radians);
  double inch_to_cm(double inch);
  double cm_to_inch(double cm);
  double inch_to_m(double inch);
  double m_to_inch(double m);

  double x_rotate_point(double x, double y, double angle, bool clockwise);
  double y_rotate_point(double x, double y, double angle, bool clockwise);

  double distance_formula(Point p1, Point p2);
  double magnitude(Point p);

  double dot_product(Point v1, Point v2);
  double determinant(Point v1, Point v2);
  double slope(Point p1, Point p2);

  bool double_click(double delay, pros::controller_digital_e_t button_1, pros::controller_digital_e_t button_2);
  Point get_perpendicular_vector(Point A, Point B, Point C);
}