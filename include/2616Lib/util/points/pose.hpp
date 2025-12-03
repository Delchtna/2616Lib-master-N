#pragma once

#include "2616Lib/util/points/point.hpp"

class Pose: public Point {
  public:
    Pose();
    Pose(double xcoord, double ycoord, double angle);
     
    double angle = 0;
};