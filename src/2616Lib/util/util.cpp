#include "main.h"
#include "util.hpp"


namespace Util {

  int sgn(double input) {
    if (input == 0)
      return 0;
    return input > 0 ? 1 : -1;
  }

  bool is_reversed(double input) {
    return input < 0;
  }

  //Clip a number between [min, max]
  double clip(double d, double max, double min) {
    const double t = d < min ? min : d;
    return t > max ? max : t;
  }

  double normalize(double angle) {
    return fmod(angle + M_PI, 2 * M_PI) - M_PI;
  }

  //Finds minimum angle between two headings
  double find_min_angle(double target_heading, double current_heading) {

    if (current_heading < 0){
      current_heading = fmod(current_heading, 2 * M_PI);
      current_heading += 2 * M_PI;
    }
    double d = fmod(std::abs(target_heading - current_heading), 2 * M_PI);
    double r = d > M_PI ? 2 * M_PI - d : d;

    double diff = target_heading - current_heading;
    // Calculate sign
    int sign = ((0 <= diff && diff <= M_PI) || (-2 * M_PI <= diff && diff <= -M_PI)) ? 1 : -1;
    r *= sign;

    return r;
  }

  double get_angle_to_point(Point target, bool away) {
    double x_error = target.x - chassis.get_pose().x;
    double y_error = target.y - chassis.get_pose().y;
    double distance = std::hypot(x_error, y_error);
    double target_angle_deg = to_deg(atan2(x_error, y_error));

    target_angle_deg += away ? 180 : 0;

    return target_angle_deg;
  }

  double wrap(double num, double range){
    num = fmod(num, range * 2);

    if (num > range){
      num -= range * 2;
    }
    return num;
  }

  double wrap_angle_90(double x) {
    x = fmod(x + 90, 360);
    if (x < 0) {
      x += 360;
    }
    return x - 180;
  }


  //Converts angle in degrees to radians
  double to_rad(double degrees) {
    return degrees * (M_PI / 180);
  }

  //Converts angle in radians to degrees
  double to_deg(double radians) {
    return radians * (180 / M_PI);
  }

  double inch_to_cm(double inch) { 
    return inch * 2.54; 
  }

  double cm_to_inch(double cm) { 
    return cm / 2.54; 
  }
  
  double inch_to_m(double inch) {
    return inch * 2.54 / 100;
  }

  double m_to_inch(double m) {
    return m / 2.54 * 100;
  }


  //https://en.wikipedia.org/wiki/Rotation_matrix#In_two_dimensions
  double x_rotate_point(double x, double y, double angle, bool clockwise) {
    return x * cos(angle) + (clockwise ? -y : y) * sin(angle);
  }
  double y_rotate_point(double x, double y, double angle, bool clockwise) {
    return (clockwise ? x : -x) * sin(angle) + y * cos(angle);
  }


  double distance_formula(Point p1, Point p2) {
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
  }

  double magnitude(Point p) {
    return std::hypot(p.x, p.y);
  }


  double dot_product(Point v1, Point v2) {
    return v1.x * v2.x + v1.y * v2.y;
  }

  double determinant(Point v1, Point v2) {
    return v1.x * v2.y - v2.x * v1.y;
  }

  double slope(Point p1, Point p2) {
    Point p = p2 - p1;
    return (p.y / p.x);
  }

  bool double_click(double delay, pros::controller_digital_e_t button_1, pros::controller_digital_e_t button_2){
    if (controller.get_digital(button_1)){
      while (delay > 0 ){
        delay -= DELAY_TIME;
        if (controller.get_digital(button_2)){
          return true;
        }
        pros::delay(10);
        }
    }
    return false;
  }

  Point get_perpendicular_vector(Point A, Point B, Point C) {
    Point AB = B - A;
    Point BC = C - B;

    Point v1 = AB / magnitude(AB);
    Point v2 = BC / magnitude(BC);

    try {
      return (v1 + v2) / magnitude(v1 + v2);
    } catch(...) {
      return Point(1, 0);
    }
  }

} //namespace Util