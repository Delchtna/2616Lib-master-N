#include "main.h"


Quintic_Bezier::Quintic_Bezier(Point point0, Point point1, Point point2, Point point3, Point point4, Point point5):
    point0(point0), point1(point1), point2(point2), point3(point3), point4(point4), point5(point5) {}

std::vector<Point> Quintic_Bezier::get_control_points() const {
  std::vector<Point> points = { point0, point1, point2, point3, point4, point5 };
  return points;
}

Point Quintic_Bezier::calc_first_derivative(double t) const {
  Point p = ((5 * std::pow(1 - t, 4) * (point1 - point0)) + (20 * t * std::pow(1 - t, 3) * (point2 - point1)) +
    (30 * std::pow(t, 2) * std::pow(1 - t, 2) * (point3 - point2)) + (20 * std::pow(t, 3) * (1 - t) * (point4 - point3)) +
    (5 * std::pow(t, 4) * (point5 - point4)));

  return p;
}

Point Quintic_Bezier::calc_second_derivative(double t) const {
  Point p = ((20 * std::pow(1 - t, 3) * (point2 - 2 * point1 + point0)) +
      (60 * t * std::pow(1 - t, 2) * (point3 - 2 * point2 + point1)) +
      (60 * std::pow(t, 2) * (1 - t) * (point4 - 2 * point3 + point2) +
      (20 * std::pow(t, 3) * (point5 - 2 * point4 + point3))));

  return p;
}

double Quintic_Bezier::calc_curvature(double t) const {
  Point first_derivative = calc_first_derivative(t);
  
  if (Util::magnitude(first_derivative) == 0) {
    return 0;
  }

  return (Util::determinant(first_derivative, calc_second_derivative(t))) / (std::pow(Util::magnitude(first_derivative), 3));
}

double Quintic_Bezier::calc_arc_length() const {
  double distance = 0;
  Point prev_point = get_point(0);
  
  for (double t = 0; t < 1.01; t += 0.01) {
    Point curr_point = get_point(t);

    distance += Util::distance_formula(curr_point, prev_point);
    prev_point = curr_point;
  }

  return distance;
}

Point Quintic_Bezier::get_point(double t) const {
   Point p = ((std::pow(1 - t, 5) * point0 + (5 * std::pow(1 - t, 4) * t * point1) + (10 * std::pow(1 - t, 3) * std::pow(t, 2) * point2) +
      (10 * std::pow(1 - t, 2) * std::pow(t, 3) * point3 + 5 * (1 - t) * std::pow(t, 4) * point4) + (std::pow(t, 5) * point5)));

  return p;
}