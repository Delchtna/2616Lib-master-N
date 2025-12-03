#include "main.h"

//TODO: clean up the control flow in all these functions because there is a *lot* of repeated code all over the place, and the nesting in some places gets *really* bad

namespace Path_Generation {

  //TODO: make points passed by const reference
  //Spline interpolation between waypoints using an arbitrary number of waypoints and initial (but not final) angle
  std::vector<Quintic_Bezier> generate_path(std::vector<Point> points, double initial_angle, double tangent_magnitude) {
    //List which holds discrete points from the quintic beziér curves
    std::vector<Quintic_Bezier> path;

    //Convert initial and final angle to radians from degrees
    initial_angle *= (M_PI / 180);

    //Iterate through waypoints and create path
    for (int curr = 0; curr < points.size() - 1; curr++) {
      path.emplace_back(calc_bezier_curve(curr, points, initial_angle, tangent_magnitude));
    }

    return path;
  }

  //Spline interpolation between waypoints using an arbitrary number of waypoints and initial/final angle
  std::vector<Quintic_Bezier> generate_path(std::vector<Point> points, double initial_angle, double final_angle, double tangent_magnitude) {
    //List which holds discrete points from the quintic beziér curves
    
    std::vector<Quintic_Bezier> path;

    //Convert initial and final angle to radians from degrees
    initial_angle *= (M_PI / 180);
    final_angle *= (M_PI / 180);

    //Iterate through waypoints and create path
    for (int curr = 0; curr < points.size() - 1; curr++) {
      path.emplace_back(calc_bezier_curve(curr, points, initial_angle, final_angle, tangent_magnitude));
    }

    return path;
  }

  //Spline interpolation between waypoints using an arbitrary number of poses
  std::vector<Quintic_Bezier> generate_path(std::vector<Pose> poses,  double tangent_magnitude) {
    //List which holds discrete points from quintic beziér curves
    std::vector<Quintic_Bezier> path;
    
    //Convert pose angles from degrees to radians
    for (Pose& p : poses) {
      p.angle = Util::to_rad(p.angle);
    }

    //Iterate through waypoints and create path
    for (int curr = 0; curr < poses.size() - 1; curr++) {
      path.emplace_back(calc_bezier_curve(curr, poses, tangent_magnitude));
    }

    return path;
  }

  std::vector<Path_Point> calculate_trajectory(std::vector<Quintic_Bezier> path, double v0, double v1, double max_v, double a_accel, double a_decel, double max_w) {
    std::vector<Path_Point> steps = {};
    double arc_length = 0;
    double s = 0.01;
    const double dT = 0.01;
    double last_velocity = 0;
    double last_w = 0;
  
    //Calculate cumulative arc length
    for (const Quintic_Bezier& curve : path) {
      arc_length += curve.calc_arc_length();
    }


    for (const Quintic_Bezier& curve : path) {
      double t = 0;

      std::vector<Point> control_points = curve.get_control_points();

      while (t < 1.0 && s < arc_length) {

        //Calculate current curvature, velocity, and point using the previous value of t
        double curvature = curve.calc_curvature(t);
        double max_reachable_velocity = (max_v * max_w) / (fabs(curvature) * max_v + max_w);         
        double velocity = std::fmin(trapezoidal_motion_profile(s, arc_length, v0, v1, max_v, a_accel, a_decel), max_reachable_velocity); 
  
        double acceleration;
        double angular_acceleration;


        double w = curvature * velocity;


        if (last_velocity == 0) {
          acceleration = velocity / dT;
          angular_acceleration = w/dT;
        } else {
          acceleration = (velocity * velocity - last_velocity * last_velocity) / (2 * last_velocity * dT);
          angular_acceleration = (w * w - last_w * last_w) / (2 * last_w * dT);
        }

        //Calculate pose to pass into steps vector
        Point point = curve.get_point(t);
        Point dydx = curve.calc_first_derivative(t);
        double angle = std::atan2(dydx.x, dydx.y);
        Pose pose(point.x, point.y, angle);
        
        //Append Path_Point to path
        steps.emplace_back(pose, curvature, w, s, velocity, acceleration, angular_acceleration);

        //Increase arc length by integrating velocity at timestep
        double dS = velocity * dT;
        s += dS;

        //Increase t by dividing change in distance in arc length by magnitude of velocity vector
        double magnitude_velocity = Util::magnitude(dydx);
        t += dS / magnitude_velocity;

 
        last_velocity = velocity;
        last_w = w;
      }
    }


    return steps;
  }


  std::vector<Path_Point> calculate_trajectory(std::vector<Quintic_Bezier> path, int num_points, double v0, double v1, double max_v, double a_accel, double a_decel, double max_w) {
    std::vector<Path_Point> steps = {};
    double arc_length = 0;
    double s = 0.01;
    const double dT = 0.01;
    double last_velocity = 0;
    double last_w = 0;
          int length = 0;
  
    //Calculate cumulative arc length
    for (const Quintic_Bezier& curve : path) {
      arc_length += curve.calc_arc_length();
    }


    for (const Quintic_Bezier& curve : path) {
      double t = 0;


      std::vector<Point> control_points = curve.get_control_points();

      while (t < 1.0 && s < arc_length && length < num_points) {

        //Calculate current curvature, velocity, and point using the previous value of t
        double curvature = curve.calc_curvature(t);
        double max_reachable_velocity = (max_v * max_w) / (fabs(curvature) * max_v + max_w);         
        double velocity = std::fmin(trapezoidal_motion_profile(s, arc_length, v0, v1, max_v, a_accel, a_decel), max_reachable_velocity); 
  
        double acceleration;
        double angular_acceleration;


        double w = curvature * velocity;


        if (last_velocity == 0) {
          acceleration = velocity / dT;
          angular_acceleration = w/dT;
        } else {
          acceleration = (velocity * velocity - last_velocity * last_velocity) / (2 * last_velocity * dT);
          angular_acceleration = (w * w - last_w * last_w) / (2 * last_w * dT);
        }

        //Calculate pose to pass into steps vector
        Point point = curve.get_point(t);
        Point dydx = curve.calc_first_derivative(t);
        double angle = std::atan2(dydx.x, dydx.y);
        Pose pose(point.x, point.y, angle);
        
        //Append Path_Point to path
        steps.emplace_back(pose, curvature, w, s, velocity, acceleration, angular_acceleration);

        //Increase arc length by integrating velocity at timestep
        double dS = velocity * dT;
        s += dS;

        //Increase t by dividing change in distance in arc length by magnitude of velocity vector
        double magnitude_velocity = Util::magnitude(dydx);
        t += dS / magnitude_velocity;

 
        last_velocity = velocity;
        last_w = w;
        length += 1;
      }
    }


    return steps;
  }
  
  std::vector<Path_Point> calc_velocity(std::vector<Path_Point> path, double v, double a) {
    //alculates velocity profile of path with max velocity and max acceleration.
    if (path.size() == 0) { return path; }
    double curvature = 0;
    
    path.back().velocity = 0;
    for (int i = path.size() - 2; i > -1; i--) {
      if (path.at(i).curvature == 0) {
        curvature = 0.001;
      } else {
        curvature = path.at(i).curvature;
      }
        
      double desired_velocity = std::min(v, 3.0 / fabs(curvature));
      double distance = Util::distance_formula(path.at(i + 1), path.at(i));
      double limited_velocity = std::pow(std::sqrt(path.at(i + 1).velocity), 2) + 2 * a * distance;

      path.at(i).velocity = std::min(desired_velocity, limited_velocity);
    }

    return path;
  }


  //Calculate control points of quintic bezier curve given an arbitrary number of points
  Quintic_Bezier calc_bezier_curve(int curr, std::vector<Pose> path, double tangent_magnitude) {  
    double magnitude_v0, magnitude_v1;
    double magnitude_vA, magnitude_vD;
    Point acc0, acc1;
    Point vA, vD;
    
    double alpha, beta, alpha0, beta0, alpha1, beta1;

    Point v0 = Point(sin(path.at(curr).angle), cos(path.at(curr).angle));
    Point v1 = Point(sin(path.at(curr + 1).angle), cos(path.at(curr + 1).angle));

    if (curr == 0) {
      //Current point is the first point of the path

      if (path.size() == 2) {
        //The path is only two points long
        magnitude_v0 = tangent_magnitude * Util::magnitude(path.at(curr + 1) - path.at(curr));

        //Multiply unit vector by magnitude to calculate first derivative of robot
        v0 = magnitude_v0 * v0;
        v1 = magnitude_v0 * v1;

        acc0 = -6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1);
        acc1 = 6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1);
      
      } else {
        //Calculate magnitude of tangent vectors, which is tangent_magnitude times the minimum of the distance to the neighboring points

        if (curr + 3 == path.size()) {
          magnitude_vD = tangent_magnitude * Util::magnitude(path.at(curr + 2) - path.at(curr + 1));
          vD = Point(sin(path.at(curr + 2).angle), cos(path.at(curr + 2).angle)) * magnitude_vD;
        } else {
          magnitude_vD = tangent_magnitude * std::min(Util::magnitude(path.at(curr + 2) - path.at(curr + 1)), Util::magnitude(path.at(curr + 3) - path.at(curr + 2)));
          vD = Point(sin(path.at(curr + 2).angle), cos(path.at(curr + 2).angle)) * magnitude_vD;
        }

        magnitude_v0 = tangent_magnitude * Util::magnitude(path.at(curr + 1) - path.at(curr));
        magnitude_v1 = tangent_magnitude * std::min(Util::magnitude(path.at(curr + 1) - path.at(curr)), Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));
        v0 = magnitude_v0 * v0;
        v1 = magnitude_v1 * v1;

        alpha = (Util::magnitude(path.at(curr + 2) - path.at(curr + 1))) /(Util::magnitude(path.at(curr + 1) - path.at(curr)) + Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));
        beta = (Util::magnitude(path.at(curr + 1) - path.at(curr))) / (Util::magnitude(path.at(curr + 1) - path.at(curr)) + Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));

        //Calculate second derivative
        acc0 = -6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1);
        acc1 = alpha * (6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1)) + beta
                    * (-6 * path.at(curr + 1) - 4 * v1 - 2 * vD + 6 * path.at(curr + 2));
      }
    
    } else if (curr == path.size() - 2) {
      //Point is second to last point on path

      if (curr - 1 == 0) {
        magnitude_vA = tangent_magnitude * Util::magnitude(path.at(curr) - path.at(curr - 1));
        vA = Point(sin(path.at(curr - 1).angle), cos(path.at(curr - 1).angle)) * magnitude_vA;
      } else {
        magnitude_vA = tangent_magnitude * std::min(Util::magnitude(path.at(curr - 2) - path.at(curr - 1)), Util::magnitude(path.at(curr) - path.at(curr - 1)));
        vA = Point(sin(path.at(curr - 1).angle), cos(path.at(curr - 1).angle)) * magnitude_vA;
      }

      magnitude_v0 = tangent_magnitude * std::min(Util::magnitude(path.at(curr) - path.at(curr - 1)), Util::magnitude(path.at(curr + 1) - path.at(curr)));
      magnitude_v1 = tangent_magnitude * Util::magnitude(path.at(curr + 1) - path.at(curr));
      v0 = magnitude_v0 * v0;
      v1 = magnitude_v1 * v1;

      alpha = (Util::magnitude(path.at(curr + 1) - path.at(curr))) /
              (Util::magnitude(path.at(curr) - path.at(curr - 1)) + Util::magnitude(path.at(curr + 1) - path.at(curr)));
      beta = (Util::magnitude(path.at(curr) - path.at(curr - 1))) / 
              (Util::magnitude(path.at(curr) - path.at(curr - 1)) + Util::magnitude(path.at(curr + 1) - path.at(curr)));

      acc0 = alpha * (6 * path.at(curr - 1) + 2 * vA + 4 * v0 - 6 * path.at(curr)) + beta
                  * (-6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1));
      acc1 = 6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1);

    } else {
      //Point is in the middle of the path

      if (curr - 1 == 0) {
        magnitude_vA = tangent_magnitude * Util::magnitude(path.at(curr) - path.at(curr - 1));
        vA = Point(sin(path.at(curr - 1).angle), cos(path.at(curr - 1).angle)) * magnitude_vA;
      } else {
        magnitude_vA = tangent_magnitude * std::min(Util::magnitude(path.at(curr - 2) - path.at(curr - 1)), Util::magnitude(path.at(curr) - path.at(curr - 1)));
        vA = Point(sin(path.at(curr - 1).angle), cos(path.at(curr - 1).angle)) * magnitude_vA;
      }

      if (curr + 2 == path.size() - 1) {
        //Two points ahead is last point on path
        magnitude_vD = tangent_magnitude * Util::magnitude(path.at(curr + 2) - path.at(curr + 1));
        vD = Point(sin(path.at(curr + 2).angle), cos(path.at(curr + 2).angle)) * magnitude_vD;
      } else {
        magnitude_vD = tangent_magnitude * std::min(Util::magnitude(path.at(curr + 2) - path.at(curr + 1)), Util::magnitude(path.at(curr + 3) - path.at(curr + 2)));
        vD = Point(sin(path.at(curr + 2).angle), cos(path.at(curr + 2).angle)) * magnitude_vD;
      }

      magnitude_v0 = tangent_magnitude * std::min(Util::magnitude(path.at(curr) - path.at(curr - 1)), Util::magnitude(path.at(curr + 1) - path.at(curr)));
      magnitude_v1 = tangent_magnitude * std::min(Util::magnitude(path.at(curr + 1) - path.at(curr)), Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));
      v0 = magnitude_v0 * v0;
      v1 = magnitude_v1 * v1;

      alpha0 = (Util::magnitude(path.at(curr + 1) - path.at(curr))) /
              (Util::magnitude(path.at(curr) - path.at(curr - 1)) + Util::magnitude(path.at(curr + 1) - path.at(curr)));
      beta0 = (Util::magnitude(path.at(curr) - path.at(curr - 1))) /
              (Util::magnitude(path.at(curr) - path.at(curr - 1)) + Util::magnitude(path.at(curr + 1) - path.at(curr)));

      alpha1 = (Util::magnitude(path.at(curr + 2) - path.at(curr + 1))) /
              (Util::magnitude(path.at(curr + 1) - path.at(curr)) + Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));
      beta1 = (Util::magnitude(path.at(curr + 1) - path.at(curr))) /
              (Util::magnitude(path.at(curr + 1) - path.at(curr)) + Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));

      acc0 = alpha0 * (6 * path.at(curr - 1) + 2 * vA + 4 * v0 - 6 * path.at(curr)) + beta0
                    * (-6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1));
      acc1 = alpha1 * (6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1)) + beta1
                    * (-6 * path.at(curr + 1) - 4 * v1 - 2 * vD + 6 * path.at(curr + 2));
    }

    v0 = round(v0, 10);
    v1 = round(v1, 10);
    acc0 = round(acc0, 10);
    acc1 = round(acc1, 10);

    //Use for debugging of control points.
    // std::cout << "v0: " <<  v0.x << " " << v0.y << std::endl;
    // std::cout << "v1: " <<  v1.x << " " << v1.y << std::endl;
    //  std::cout << "vd: " <<  vD.x << " " << vD.y << std::endl;
    // std::cout << "acc0: " <<  acc0.x << " " << acc0.y << std::endl;
    // std::cout << "acc1: " <<  acc1.x << " " << acc1.y << std::endl;
    // std::cout << "alpha: " << alpha << std::endl;
    //  std::cout << "beta: " << beta << std::endl;

    //Calculate control points based on first and second derivative
    Point point0 = path.at(curr);
    Point point5 = path.at(curr + 1);
    Point point1 = (1 / 5.0) * v0 + point0;
    Point point2 = (1 / 20.0) * acc0 + 2 * point1 - point0;
    Point point4 = point5 - (1 / 5.0) * v1;
    Point point3 = (1 / 20.0) * acc1 + 2 * point4 - point5;

    //Return Quintic Bezier for current point
    return Quintic_Bezier(point0, point1, point2, point3, point4, point5);
  }

  //Calculate control points of quintic bezier curve given an arbitrary number of points and initial/final angle
  Quintic_Bezier calc_bezier_curve(int curr, std::vector<Point> path, double initial_angle, double final_angle, double tangent_magnitude) {
    double magnitude_v0, magnitude_v1;
    double magnitude_vA, magnitude_vD;
    Point acc0, acc1;
    Point v0, v1;
    Point vA, vD;


    double alpha, beta, alpha0, beta0, alpha1, beta1;

    if (curr == 0) {
      if (path.size() == 2) {
        magnitude_v0 = tangent_magnitude * Util::magnitude(path.at(curr + 1) - path.at(curr));

        v0 = Point(sin(initial_angle), cos(initial_angle)) * magnitude_v0;

        //If final angle exists, use final angle to calculate first derivative. Otherwise, match direction of straight line connection from previous point

        v1 = Point(sin(final_angle), cos(final_angle)) * magnitude_v0;

        acc0 = -6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1);
        acc1 = 6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1);
      
      } else {

        if (curr + 2 == path.size() - 1) {
          magnitude_vD = tangent_magnitude * Util::magnitude(path.at(curr + 2) - path.at(curr + 1));
          vD = Point(sin(final_angle), cos(final_angle)) * magnitude_vD;

        } else {
          magnitude_vD = tangent_magnitude * std::min(Util::magnitude(path.at(curr + 2) - path.at(curr + 1)), Util::magnitude(path.at(curr + 3) - path.at(curr + 2)));
          vD = Util::get_perpendicular_vector(path.at(curr + 1), path.at(curr + 2), path.at(curr + 3)) * magnitude_vD;
        }

        magnitude_v0 = tangent_magnitude * Util::magnitude(path.at(curr + 1) - path.at(curr));
        magnitude_v1 = tangent_magnitude * std::min(Util::magnitude(path.at(curr + 1) - path.at(curr)), Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));

        Point p = path.at(curr + 1)-path.at(curr);
        v0 = Point(sin(initial_angle), cos(initial_angle)) * magnitude_v0;
        //Calculate vector perpendicular to the angle bisector of surrounding points
        v1 = Util::get_perpendicular_vector(path.at(curr), path.at(curr + 1), path.at(curr + 2)) * magnitude_v1;

        alpha = (Util::magnitude(path.at(curr + 2) - path.at(curr + 1))) /
                (Util::magnitude(path.at(curr + 1) - path.at(curr)) + Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));
        beta = (Util::magnitude(path.at(curr + 1) - path.at(curr))) /
                (Util::magnitude(path.at(curr + 1) - path.at(curr)) + Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));

        acc0 = -6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1);
        acc1 = alpha * (6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1)) + beta
                    * (-6 * path.at(curr + 1) - 4 * v1 - 2 * vD + 6 * path.at(curr + 2));
      }

    } else if (curr == path.size() - 2) {
      if (curr - 1 == 0) {
        magnitude_vA = tangent_magnitude * Util::magnitude(path.at(curr) - path.at(curr - 1));
        vA = Point(sin(initial_angle), cos(initial_angle)) * magnitude_vA;
      } else {
        magnitude_vA = tangent_magnitude * std::min(Util::magnitude(path.at(curr - 2) - path.at(curr - 1)), Util::magnitude(path.at(curr) - path.at(curr - 1)));
        vA = Util::get_perpendicular_vector(path.at(curr - 2), path.at(curr - 1), path.at(curr)) * magnitude_vA;
      }

      magnitude_v0 = tangent_magnitude * std::min(Util::magnitude(path.at(curr) - path.at(curr - 1)), Util::magnitude(path.at(curr + 1) - path.at(curr)));
      magnitude_v1 = tangent_magnitude * Util::magnitude(path.at(curr + 1) - path.at(curr));
      v0 = Util::get_perpendicular_vector(path.at(curr - 1), path.at(curr), path.at(curr + 1)) * magnitude_v0;
      v1 = Point(sin(final_angle), cos(final_angle)) * magnitude_v1;

      alpha = (Util::magnitude(path.at(curr + 1) - path.at(curr))) /
              (Util::magnitude(path.at(curr) - path.at(curr - 1)) + Util::magnitude(path.at(curr + 1) - path.at(curr)));
      beta = (Util::magnitude(path.at(curr) - path.at(curr - 1))) /
              (Util::magnitude(path.at(curr) - path.at(curr - 1)) + Util::magnitude(path.at(curr + 1) - path.at(curr)));
      acc0 = alpha * (6 * path.at(curr - 1) + 2 * vA + 4 * v0 - 6 * path.at(curr)) + beta
                  * (-6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1));
      acc1 = 6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1);

    } else {
      if (curr - 1 == 0) {
        magnitude_vA = tangent_magnitude * Util::magnitude(path.at(curr) - path.at(curr - 1));
        vA = Point(sin(initial_angle), cos(initial_angle)) * magnitude_vA;
      } else {
        magnitude_vA = tangent_magnitude * std::min(Util::magnitude(path.at(curr - 2) - path.at(curr - 1)), Util::magnitude(path.at(curr) - path.at(curr - 1)));
        vA = Util::get_perpendicular_vector(path.at(curr - 2), path.at(curr - 1), path.at(curr)) * magnitude_vA;
      }

      if (curr + 2 == path.size() - 1) {
        //Two points ahead is last point on path
        magnitude_vD = tangent_magnitude * Util::magnitude(path.at(curr + 2) - path.at(curr + 1));
        vD = Point(sin(final_angle), cos(final_angle)) * magnitude_vD;
      } else {
        magnitude_vD = tangent_magnitude * std::min(Util::magnitude(path.at(curr + 2) - path.at(curr + 1)), Util::magnitude(path.at(curr + 3) - path.at(curr + 2)));
        vD = Util::get_perpendicular_vector(path.at(curr + 1), path.at(curr + 2), path.at(curr + 3)) * magnitude_vD;
      }

      magnitude_v0 = tangent_magnitude * std::min(Util::magnitude(path.at(curr) - path.at(curr - 1)), Util::magnitude(path.at(curr + 1) - path.at(curr)));
      magnitude_v1 = tangent_magnitude * std::min(Util::magnitude(path.at(curr + 1) - path.at(curr)), Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));
      v0 = Util::get_perpendicular_vector(path.at(curr - 1), path.at(curr), path.at(curr + 1)) * magnitude_v0;
      v1 = Util::get_perpendicular_vector(path.at(curr), path.at(curr + 1), path.at(curr + 2)) * magnitude_v1;

      alpha0 = (Util::magnitude(path.at(curr + 1) - path.at(curr))) /
              (Util::magnitude(path.at(curr) - path.at(curr - 1)) + Util::magnitude(path.at(curr + 1) - path.at(curr)));
      beta0 = (Util::magnitude(path.at(curr) - path.at(curr - 1))) /
              (Util::magnitude(path.at(curr) - path.at(curr - 1)) + Util::magnitude(path.at(curr + 1) - path.at(curr)));

      alpha1 = (Util::magnitude(path.at(curr + 2) - path.at(curr + 1))) /
              (Util::magnitude(path.at(curr + 1) - path.at(curr)) + Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));
      beta1 = (Util::magnitude(path.at(curr + 1) - path.at(curr))) /
              (Util::magnitude(path.at(curr + 1) - path.at(curr)) + Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));

      acc0 = alpha0 * (6 * path.at(curr - 1) + 2 * vA + 4 * v0 - 6 * path.at(curr)) + beta0
                    * (-6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1));
      acc1 = alpha1 * (6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1)) + beta1
                    * (-6 * path.at(curr + 1) - 4 * v1 - 2 * vD + 6 * path.at(curr + 2));
    }

    v0 = round(v0, 10);
    v1 = round(v1, 10);
    acc0 = round(acc0, 10);
    acc1 = round(acc1, 10);
    //Calculate control points based on first and second derivative
    Point point0 = path.at(curr);
    Point point5 = path.at(curr + 1);
    Point point1 = (1 / 5.0) * v0 + point0;
    Point point2 = (1 / 20.0) * acc0 + 2 * point1 - point0;
    Point point4 = point5 - (1 / 5.0) * v1;
    Point point3 = (1 / 20.0) * acc1 + 2 * point4 - point5;

    //Return Quintic Bezier for current point
    return Quintic_Bezier(point0, point1, point2, point3, point4, point5);
  }

  //Calculate control points of quintic bezier curve given an arbitrary number of points and initial (but not final) angle
  Quintic_Bezier calc_bezier_curve(int curr, std::vector<Point> path, double initial_angle, double tangent_magnitude) {
    double magnitude_v0, magnitude_v1;
    double magnitude_vA, magnitude_vD;
    Point v0, v1;
    Point acc0, acc1;
    Point vA, vD;
    Point Point_vD;

    double alpha, beta, alpha0, beta0, alpha1, beta1;


    if (curr == 0) {
      if (path.size() == 2) {
        magnitude_v0 = tangent_magnitude * Util::magnitude(path.at(curr + 1) - path.at(curr));
        v0 = Point(sin(initial_angle), cos(initial_angle)) * magnitude_v0;

        //If final angle exists, use final angle to calculate first derivative. Otherwise, match direction to straight line connection from previous point

        v1 = ((path.at(curr + 1) - path.at(curr)) / (Util::magnitude(path.at(curr + 1) - path.at(curr)))) * magnitude_v0;

        acc0 = -6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1);
        acc1 = 6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1);

      } else {

        if (curr + 2 == path.size() - 1) {
          magnitude_vD = tangent_magnitude * Util::magnitude(path.at(curr + 2) - path.at(curr + 1));
          vD = (path.at(curr + 2) - path.at(curr + 1)) / (Util::magnitude(path.at(curr + 2) - path.at(curr + 1))) * magnitude_vD;
        } else {
          magnitude_vD = tangent_magnitude * std::min(Util::magnitude(path.at(curr + 2) - path.at(curr + 1)), Util::magnitude(path.at(curr + 3) - path.at(curr + 2)));
          vD = Util::get_perpendicular_vector(path.at(curr + 1), path.at(curr + 2), path.at(curr + 3)) * magnitude_vD;
        } //TODO: I moved this closing bracket to match the other overloads of this function, so make sure this works correctly since the control flow is changed   -Max

        magnitude_v0 = tangent_magnitude * Util::magnitude(path.at(curr + 1) - path.at(curr));
        magnitude_v1 = tangent_magnitude * std::min(Util::magnitude(path.at(curr + 1) - path.at(curr)), Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));
        v0 = Point(sin(initial_angle), cos(initial_angle)) * magnitude_v0;

        //Calculate vector perpendicular to the angle bisector of surrounding points
        v1 = Util::get_perpendicular_vector(path.at(curr), path.at(curr + 1), path.at(curr + 2)) * magnitude_v1;

        alpha = (Util::magnitude(path.at(curr + 2) - path.at(curr + 1))) /
                (Util::magnitude(path.at(curr + 1) - path.at(curr)) + Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));
        beta = (Util::magnitude(path.at(curr + 1) - path.at(curr))) /
                (Util::magnitude(path.at(curr + 1) - path.at(curr)) + Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));

        acc0 = -6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1);
        acc1 = alpha * (6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1)) + beta
                    * (-6 * path.at(curr + 1) - 4 * v1 - 2 * vD + 6 * path.at(curr + 2));
      }

    } else if (curr == path.size() - 2) {
      if (curr - 1 == 0) {
        magnitude_vA = tangent_magnitude * Util::magnitude(path.at(curr) - path.at(curr - 1));
        vA = Point(sin(initial_angle), cos(initial_angle)) * magnitude_vA;
      } else {
        magnitude_vA = tangent_magnitude * std::min(Util::magnitude(path.at(curr - 2) - path.at(curr - 1)), Util::magnitude(path.at(curr) - path.at(curr - 1)));
        vA = Util::get_perpendicular_vector(path.at(curr - 2), path.at(curr - 1), path.at(curr)) * magnitude_vA;
      }
      
      magnitude_v0 = tangent_magnitude * std::min(Util::magnitude(path.at(curr) - path.at(curr - 1)), Util::magnitude(path.at(curr + 1) - path.at(curr)));
      magnitude_v1 = tangent_magnitude * Util::magnitude(path.at(curr + 1) - path.at(curr));
      v0 = Util::get_perpendicular_vector(path.at(curr - 1), path.at(curr), path.at(curr + 1)) * magnitude_v0;
      v1 = (path.at(curr + 1) - path.at(curr)) / (Util::magnitude(path.at(curr + 1) - path.at(curr))) * magnitude_v1;

      alpha = (Util::magnitude(path.at(curr + 1) - path.at(curr))) /
              (Util::magnitude(path.at(curr) - path.at(curr - 1)) + Util::magnitude(path.at(curr + 1) - path.at(curr)));
      beta = (Util::magnitude(path.at(curr) - path.at(curr - 1))) /
              (Util::magnitude(path.at(curr) - path.at(curr - 1)) + Util::magnitude(path.at(curr + 1) - path.at(curr)));
      acc0 = alpha * (6 * path.at(curr - 1) + 2 * vA + 4 * v0 - 6 * path.at(curr)) + beta
                  * (-6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1));
      acc1 = 6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1);

    } else {
      if (curr - 1 == 0) {
        magnitude_vA = tangent_magnitude * Util::magnitude(path.at(curr) - path.at(curr - 1));
        vA = Point(sin(initial_angle), cos(initial_angle)) * magnitude_vA;
      } else {
        magnitude_vA = tangent_magnitude * std::min(Util::magnitude(path.at(curr - 2) - path.at(curr - 1)), Util::magnitude(path.at(curr) - path.at(curr - 1)));
        vA = Util::get_perpendicular_vector(path.at(curr - 2), path.at(curr - 1), path.at(curr)) * magnitude_vA;
      }
      
      if (curr + 2 == path.size() - 1) {
        //Two points ahead is last point on path
        magnitude_vD = tangent_magnitude * Util::magnitude(path.at(curr + 2) - path.at(curr + 1));
        vD = (path.at(curr + 2) - path.at(curr + 1)) / (Util::magnitude(path.at(curr + 2) - path.at(curr + 1))) * magnitude_vD;
      } else {
        magnitude_vD = tangent_magnitude * std::min(Util::magnitude(path.at(curr + 2) - path.at(curr + 1)), Util::magnitude(path.at(curr + 3) - path.at(curr + 2)));
        vD = Util::get_perpendicular_vector(path.at(curr + 1), path.at(curr + 2), path.at(curr + 3)) * magnitude_vD;
      }

      magnitude_v0 = tangent_magnitude * std::min(Util::magnitude(path.at(curr) - path.at(curr - 1)), Util::magnitude(path.at(curr + 1) - path.at(curr)));
      magnitude_v1 = tangent_magnitude * std::min(Util::magnitude(path.at(curr + 1) - path.at(curr)), Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));
      v0 = Util::get_perpendicular_vector(path.at(curr - 1), path.at(curr), path.at(curr + 1)) * magnitude_v0;
      v1 = Util::get_perpendicular_vector(path.at(curr), path.at(curr + 1), path.at(curr + 2)) * magnitude_v1;

      alpha0 = (Util::magnitude(path.at(curr + 1) - path.at(curr))) /
              (Util::magnitude(path.at(curr) - path.at(curr - 1)) + Util::magnitude(path.at(curr + 1) - path.at(curr)));
      beta0 = (Util::magnitude(path.at(curr) - path.at(curr - 1))) /
              (Util::magnitude(path.at(curr) - path.at(curr - 1)) + Util::magnitude(path.at(curr + 1) - path.at(curr)));

      alpha1 = (Util::magnitude(path.at(curr + 2) - path.at(curr + 1))) /
              (Util::magnitude(path.at(curr + 1) - path.at(curr)) + Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));
      beta1 = (Util::magnitude(path.at(curr + 1) - path.at(curr))) /
              (Util::magnitude(path.at(curr + 1) - path.at(curr)) + Util::magnitude(path.at(curr + 2) - path.at(curr + 1)));

      acc0 = alpha0 * (6 * path.at(curr - 1) + 2 * vA + 4 * v0 - 6 * path.at(curr)) + beta0
                    * (-6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1));
      acc1 = alpha1 * (6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1)) + beta1
                    * (-6 * path.at(curr + 1) - 4 * v1 - 2 * vD + 6 * path.at(curr + 2));
    }

    v0 = round(v0, 10);
    v1 = round(v1, 10);
    acc0 = round(acc0, 10);
    acc1 = round(acc1, 10);
    //Calculate control points based on first and second derivative
    Point point0 = path.at(curr);
    Point point5 = path.at(curr + 1);
    Point point1 = (1 / 5.0) * v0 + point0;
    Point point2 = (1 / 20.0) * acc0 + 2 * point1 - point0;
    Point point4 = point5 - (1 / 5.0) * v1;
    Point point3 = (1 / 20.0) * acc1 + 2 * point4 - point5;

    //Return Quintic Bezier for current point
    return Quintic_Bezier(point0, point1, point2, point3, point4, point5);
  }



double trapezoidal_motion_profile(double distance, double total_dist, double v0, double v1, double v_max, double a_accel, double a_decel) {

    double cruise_velocity = std::sqrt((2 * total_dist * a_accel * a_decel + a_decel * v0 * v0 - a_accel * v1 * v1) / (a_decel - a_accel));
    cruise_velocity = std::min(v_max, cruise_velocity);

    double v_forward = std::sqrt(v0 * v0 + 2 * a_accel * distance);
    double v_back = std::sqrt(v1 * v1 - 2 * a_decel * (total_dist - distance));


    return std::min({cruise_velocity, v_forward, v_back});
}

} //namespace Path_Generation