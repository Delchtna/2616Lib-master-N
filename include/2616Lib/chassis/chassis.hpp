#pragma once

#include "api.h"
#include "2616Lib/util/PID.hpp"
#include "2616Lib/chassis/odometry/tracking_wheel.hpp"
#include "2616Lib/chassis/motion/motion_profiling.hpp"
#include <atomic>


class Chassis {
  public:
    // ************************ CONSTRUCTOR ************************

    //Main chassis constructor. Odom type is determined by which combination of tracking wheels are enabled
    Chassis(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, std::vector<int> imu_sensor_ports,
            std::vector<Tracking_Wheel> trackers, bool disable_odom = false);


    // ************************ PUBLIC VARIABLES / METHODS ************************

    //Drive motors and sensors
    std::vector<pros::Motor> left_motors;
    std::vector<pros::Motor> right_motors;
    std::vector<pros::IMU> imu_sensors;

    //Tracking Wheel objects
    Tracking_Wheel perpendicular_tracker, left_tracker, right_tracker;
    
    //PID objects
    PID drive_PID;
    PID turn_PID;
    PID heading_PID;
    PID arc_PID;
    // double last_velocity;

    Motion_Profiling path_traverser;

    //Get and set the chassis drive mode during programmed movements
    enum class e_drive_mode { STANDBY, DRIVE, TURN, ARC, MOTION_PROFILING };
    e_drive_mode get_drive_mode();
    void set_drive_mode(e_drive_mode new_mode);
  

    //Odom position setters
    void set_odom_position(double x, double y, double angle);
    void set_odom_position(Pose pose);
    void set_odom_x(double x);
    void set_odom_y(double y);
    void set_odom_angle(double angle);

    //Get the current robot pose
    Pose& get_pose();

    void set_bound(double close_bound);

    // ************************ MOVEMENT METHODS ************************

    //Move the left side and right side drive motors at specific speeds - mainly used internally
    void set_tank(double left, double right);

    //Driving straight forward or backward
    void drive(double distance, float speed, float min_speed = 0);
    void drive(Point point, float speed, bool drive_reversed = false, float min_speed = 0);
    void drive(Pose pose, float speed, bool drive_reversed = false,float min_speed = 0);

    //Turn to face an angle, point, or pose
    void turn(double target, float speed);
    void turn(Point point, float speed, bool away = false);
    void turn(Pose pose, float speed, bool away = false);
    
    //When making an arc movement, this represents the direction that the robot will turn
    enum class e_arc_direction { LEFT, RIGHT };
    //Turn to an angle by spinning only one side of the drive
    void arc(double target, e_arc_direction direction, float speed);

    void move_to_point(Point point, float speed, float min_speed = 0);
    void move_to_point(double xcoord, double ycoord, float speed, float min_speed = 0);
    void move_to_pose(Pose pose, float speed, bool turn_to_final_angle, float min_speed =0 );
  
    //Use motion profiling to follow a path of points - ADVANCED USERS ONLY
    void motion_profiling(std::vector<Pose> path, double tangent_magnitude,  double v1, double v_max, double a_accel, double a_decel, double w, bool reversed=false);
    void motion_profiling(std::vector<Point> path, double final_angle, double tangent_magnitude, double v1, double v_max, double a_accel, double a_decel, double w, bool reversed=false);

    // ************************ USER INPUT METHODS ************************

    //Control driver experience
    void set_active_brake_power(double kP, int threshold);
    void set_joystick_curve(double scale, std::string curve_type = "red");
    void prefer_wheel_calculated_odom_angle(bool prefer_wheel_calculation);
    void tank_drive();
    void arcade_drive(bool flipped = false);


    // ************************ OTHER METHODS ************************

    //Odom related methods
    void start_tasks();
    void wait_drive();
    double get_imu_rotation();
    void imu_reset();
    bool imu_is_calibrating();
    void reset_drive_sensors();

    //Misc
    double get_distance(Point target_pose);
    static uint32_t get_auton_selector_button_color();
    void set_brake_mode(pros::motor_brake_mode_e_t brake_type);
    void ramp_voltage_test(double volts_per_second);
    void constant_voltage_test(double volts);
    void turn_test(double volts);

    pros::Mutex odom_mutex;
    pros::Mutex drive_mutex;





  private:
    //Describes the type of odometry used by this chassis
    enum class e_odom_type { NONE, THREE_WHEEL, TWO_WHEEL, SINGLE_PARALLEL_IMU, DOUBLE_PARALLEL_IMU };
    //Odom type of this chassis - should never change after initialization
    const e_odom_type ODOM_TYPE;
    //Determine the type of odometry used by this chassis based on which tracking wheels are enabled
    e_odom_type determine_odom_type();
    
    //This instance of Pose contains the current robot position, which is updated using odometry. ANGLE IS STORED HERE IN RADIANS
    Pose robot_pose;
    
    e_drive_mode drive_mode;
    e_arc_direction current_arc_direction;

    //Internal PID movement variables
    Pose target_pose;
    double target_angle;
    int starting_x_error_sgn;
    int starting_y_error_sgn;
    int reversed;
    double close_bound;

    //Motion profiling
    friend class Motion_Profiling;
    std::vector<Path_Point> trajectory;
    int current_index;
    
    //Odometry
    std::atomic<bool> update_odom; //Atomic because used in multiple concurrent tasks
    bool is_imu_error = false; //Whether any of the IMUs return an error value while getting their rotation
    bool prefer_calculated_odom_angle = false; //True means odom angle should be calculated using the perpendicular tracking wheels (if possible), false means odom angle should be calculated using IMUs (if possible)
    void odom_task_func();
    void movement_task_func();
    
    //Movement tasks
    void drive_pid_task();
    void turn_pid_task();
    void arc_pid_task();
    void motion_profiling_task();

    //Brain graphics methods and tasks
    void brain_printing_task_func();
    void update_brain_display();
    void print_pose_text();
    void draw_field_graphic();
    void draw_bot_on_field_graphic();
    void print_motor_temps();

    //Active brake
    int active_brake_threshold;
    double active_brake_kp;
    double left_brake_set_point;
    double right_brake_set_point;

    //Joystick curve
    double joystick_curve_scale;
    std::string joystick_curve_type = "red";
    double joystick_curve_function(double input);
}; 