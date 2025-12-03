#pragma once

#include "2616Lib/util/points/pose.hpp"
#include <vector>

class Quintic_Bezier {
  public:
    Quintic_Bezier(Point point0, Point point1, Point point2, Point point3, Point point4, Point point5);
    
    std::vector<Point> get_control_points() const;
    Point calc_first_derivative(double t) const;
    Point calc_second_derivative(double t) const;
    double calc_curvature(double t) const;
    double calc_arc_length() const;
    Point get_point(double t) const;

    Point point0;
    Point point1;
    Point point2;
    Point point3;
    Point point4;
    Point point5;
};