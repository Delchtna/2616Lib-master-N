#pragma once

#include "api.h"

class Tracking_Wheel {
  public:
    //Describes the type (location) of a tracking wheel, which is used to determine which tracking wheels are enabled in the chassis constructor
    enum class e_tracker_type { PERPENDICULAR, LEFT, RIGHT };

    Tracking_Wheel();
    Tracking_Wheel(e_tracker_type type, pros::Rotation rotation, bool reversed, double wheel_diameter, double offset, double ratio = 1);
    Tracking_Wheel(e_tracker_type type, pros::Motor motor, double wheel_diameter, double offset, double ratio = 1);
    Tracking_Wheel(e_tracker_type type, pros::ADIEncoder encoder, double wheel_diameter, double offset, double ratio = 1);


    double get_value_ticks();
    double get_value_inches();
    std::pair<double, bool> get_status();

    double convert_ticks_to_inches(double ticks);

    void reset_position();

    bool is_enabled();

    double get_offset();

    e_tracker_type get_type();
    static Tracking_Wheel find_by_type(std::vector<Tracking_Wheel> trackers, e_tracker_type type, bool disable_odom);


  private:
    bool disabled = false;

    e_tracker_type tracker_type;

    double wheel_size;
    double ticks_per_rev;
    double offset;
    double ratio;
    double error_value;

    void set_constants(e_tracker_type type, double wheel_size, double ticks_per_rev, double offset, double ratio, double error_value);

    std::function<double()> get_value_func;
    std::function<void()> reset_func;
};