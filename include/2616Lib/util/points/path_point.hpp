#pragma once

#include "2616Lib/util/points/pose.hpp"

class Path_Point: public Point {
  public:
    Path_Point();
    Path_Point(Pose pose, double curvature, double angular_velocity, double s = 0, double velocity = 0, double acceleration = 0, double angular_acceleration=0);
    Path_Point(double x, double y, double angle, double curvature, double angular_velocity, double s = 0, double velocity = 0, double acceleration = 0, double angular_acceleration=0);
    
    Pose pose;
    double curvature;
    double acceleration;
    double angular_acceleration;
    double s;
    double angular_velocity;
    double velocity;
};