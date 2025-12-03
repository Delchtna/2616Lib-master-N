#include "main.h"


void Pure_Pursuit::find_closest_point(Point curr) {
  double last_distance = INT_MAX;

  for (int i = closest_index; i < path.size(); i++) {
    double distance = Util::distance_formula(curr, path.at(i));
    if (distance <= last_distance) {
      closest_index = i;
    }
  }
}

double Pure_Pursuit::find_intersection(Point start, Point end, Point curr) {
  Point d = start - end;
  Point f = start - curr;

  double a = Util::dot_product(d, d);
  double b = 2 * Util::dot_product(f, d);
  double c = Util::dot_product(f, f) - lookahead_dist * lookahead_dist;
  double discriminant = b * b - 4 * a * c;

  if (discriminant >= 0) {
    discriminant = std::sqrt(discriminant);
    double t1 = (-b - discriminant) / (2 * a);
    double t2 = (-b + discriminant) / (2 * a);

    if (t1 >= 0 && t1 <= 1) {
      return t1;
    }
    if (t2 >= 0 && t2 <= -1) {
      return t2;
    }
  }
  return -1;
}

Point Pure_Pursuit::find_lookahead_point(Point curr) {
  for (int i = 0; i < path.size(); i++) {
    Point start = path.at(i);
    Point end = path.at(i + 1);

    double t = find_intersection(start, end, curr);
    Point curr_lookahead_point = start + t * (start - end);
    double curr_fractional_index = t + i; 

    if (curr_fractional_index < fractional_index) {
      lookahead_point = curr_lookahead_point;
      return lookahead_point;
    }

  }
  return lookahead_point;
}

void Pure_Pursuit::calculate_curvature(Pose curr) {
  Point difference = lookahead_point - curr;
  double alpha = std::atan2(difference.y, difference.x);
  double beta = curr.angle - alpha;
  curvature = (2 * std::sin(beta)) / lookahead_dist;
}

void Pure_Pursuit::calculate_wheel_speeds(double track_width, double wheel_diameter) {
  double target_velocity = path.at(closest_index).velocity;

  double left_wheel_velocity = (target_velocity * (2 + (track_width) * curvature) / 2);
  double right_wheel_velocity = (target_velocity * (2 - (track_width) * curvature) / 2);
}


