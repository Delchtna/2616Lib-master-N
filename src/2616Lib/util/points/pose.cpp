#include "main.h"

Pose::Pose():
  Point(0, 0), angle(0) {}

Pose::Pose(double x, double y, double angle):
  Point(x, y), angle(angle) {}

