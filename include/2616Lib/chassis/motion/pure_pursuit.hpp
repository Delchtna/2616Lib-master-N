#pragma once

#include "2616Lib/util/points/path_point.hpp"
#include "2616Lib/util/points/pose.hpp"
#include <vector>

class Pure_Pursuit {
  private:
    std::vector<Path_Point> path;
    Point lookahead_point;
    double lookahead_dist;
    double closest_index;
    double fractional_index;
    double left_wheel_velocity;
    double right_wheel_velocity;
    double curvature;
    double k;

    void find_closest_point(Point curr);
    double find_intersection(Point start, Point end, Point curr);
    Point find_lookahead_point(Point curr);
    void calculate_curvature(Pose curr);
    void calculate_wheel_speeds(double track_width, double wheel_diameter);
};