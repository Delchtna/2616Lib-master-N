#pragma once

#include "2616Lib/util/points/path_point.hpp"
#include "2616Lib/util/simple_moving_average.hpp"
#include <string>
#include <vector>

class Motion_Profiling {
  public:
    Motion_Profiling();
    Motion_Profiling(double kP, double kV, double kA);

    std::vector<Path_Point> trajectory;
    std::vector<Point> point_path;
    std::vector<Pose> pose_path;

    enum class e_path_type {POINT, POSE, STANDBY};
    e_path_type path_type;
    
    Path_Point current_point;
    int current_index;
    double left_wheel_velocity, right_wheel_velocity;
    double delta_v, delta_w;
    double acceleration;
    double prev_velocity;
    double prev_left_wheel_velocity;
    double prev_right_wheel_velocity;
    double current_v;
    bool running;
    double current_angle, last_angle;
    
    std::vector<std::string> log_data;

    double starting_position;
    double kP, kV, kA, kS;
    double zeta, beta;
    double ratio, track_width, wheel_diameter;
    bool at_end;
    double test;
 
    double minimum_time_to_update;
    double distance_threshold;
    double exit_distance;
    int time_from_last_update  = 0;

    double final_angle, tangent_magnitude;
    double v1;
    double v_max;
    double a_accel, a_decel;
    double w;
    double reversed;

    int count;

    Simple_Moving_Average left_wheel_sma, right_wheel_sma;
    int sma_period = 5;

    //Methods
    void set_trajectory(std::vector<Path_Point>& trajectory);
    void update_trajectory();

    void print_logs();
    void update_path(std::vector<Point>& path);
    void update_path(std::vector<Pose>& path);
    void update_current_point();
    void check_if_at_end();
    bool check_if_update();
    void set_software_constants(double kP, double kV, double kA, double kS, double zeta, double beta, double  minimum_time_to_update, double distance_threshold, double exit_distance, int sma_period = 5);
    void set_hardware_constants(double ratio, double track_width, double wheel_diameter);
    void set_constraints( double final_angle, double tangent_magnitude,  double v1, double v_max, double a_accel, double a_decel, double w, bool reversed);
    void calculate_wheel_speeds();

    void run_ramsete_controller();
};

