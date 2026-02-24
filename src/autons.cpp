#include "2616Lib/chassis/chassis.hpp"
#include "2616Lib/subsystems/intake.hpp"
#include "2616Lib/subsystems/pistons.hpp"
#include "2616Lib/util/util.hpp"
#include "globals.hpp"
#include "main.h"

// ************* Setting Movement Constants *************

void set_default_movement_constants() {
  //PID constants
  //Your robot uses PIDs to control the drive motors during autonomous to smoothly move from a starting position to a target. They are used for driving straight, turning, and more.
  //For more information on how PID controllers work, reference this document made by George Gillard:   https://georgegillard.com/resources/documents.
  //EZ-Template has a good tutorial for tuning PIDs, which can be found here:   https://ez-robotics.github.io/EZ-Template/Tutorials/tuning_constants#tuning-pid.
  //On EZ Template's website, IGNORE THE CODE ITSELF, since 2616Lib's code isn't compatible with EZ's. However, the information about tuning a PID is still very useful. Also, 2616Lib doesn't use slew, so you can ignore that section on EZ's website.
  chassis.drive_PID.set_constants(13, 0, 31.5, 0);
  chassis.turn_PID.set_constants(119,2.2, 740, 0);
  chassis.heading_PID.set_constants(120, 0, 0, Util::to_rad(2.5));
  chassis.arc_PID.set_constants(250.0, 35, 85, Util::to_rad(2.5));
  chassis.set_bound(2);

  //PID exit conditions
  //These determine when a PID should stop running. If the PID's error is less than `small_error` (in either inches or radians) for at least `exit_time` milliseconds, the PID will stop running.
  chassis.drive_PID.set_exit_conditions(1.5, 100, 5, 800, 0.01, 500);
  chassis.turn_PID.set_exit_conditions(Util::to_rad(2.5), 250, Util::to_rad(7), 400, Util::to_rad(1) * 0.01, 500);
  chassis.arc_PID.set_exit_conditions(Util::to_rad(2), 500, Util::to_rad(5), 500, Util::to_rad(1) * 0.01, 500);

  //Motion profiling - ADVANCED USERS ONLY
  chassis.path_traverser.set_software_constants(.35, 1.54, .251, 4, .25, 16, 0, 99, .5, 3);
  chassis.path_traverser.set_hardware_constants(.75, 11, 3.25);
}

//If you want to change your PID constants temporarily for specific types of movements, you can add more functions like these, then call them whenver they're needed in your autons. When you're done with that type of movement, call set_default_movement_constants() again.
void set_close_movements_constants() {
  chassis.drive_PID.set_exit_conditions(2, 700, 3, 1000, 0.01, 500);
}

void set_ptp_constants(){
    //These determine when a PID should stop running. If the PID's error is less than `small_error` (in either inches or radians) for at least `exit_time` milliseconds, the PID will stop running.
  chassis.drive_PID.set_exit_conditions(5, 100, 10, 1000, 0.01, 500);
  chassis.turn_PID.set_exit_conditions(Util::to_rad(2), 100, Util::to_rad(5), 500, Util::to_rad(1) * 0.01, 500);
  chassis.arc_PID.set_exit_conditions(Util::to_rad(2), 100, Util::to_rad(5), 500, Util::to_rad(1) * 0.01, 500);
}

/*****************************************************************************/
/*                                                                           */
/*     Write all of your autonomous routines (including both match autons    */
/*      and programming skills routes) below! Always create a new method     */
/*         for each separate routine, then declare the method name in        */
/*    autons.hpp, and add it to the auton selector in main.cpp to test it!   */
/*                                                                           */
/*****************************************************************************/


void drive_example() {
  //ALWAYS the robot's odom position based on where the physical robot is on the field! The center of the field is (0, 0).
  chassis.set_odom_position(0, 0, 0);
  
  //When using `.drive()`, the robot will move straight forward or backward.
  //The first parameter for `.drive()` is either the distance to move forward (in inches), or a Point/Pose to move to.
  //The second parameter is the max speed that the robot will move.

  chassis.set_tank(80,80);
  pros::delay(1000);
  chassis.set_tank(0,0);  
}

void turn_example() {
  //ALWAYS the robot's odom position based on where the physical robot is on the field! The center of the field is (0, 0).
  chassis.set_odom_position(10, 25, 90);

  //When using `.turn()`, the robot will turn toward an angle or a point.
  //The first parameter for `.turn()` is the angle relative to the robot (in degrees) to face. For example, a value of 90 means turning 90 degrees, not turning to face the angle 90.
  //The second parameter is the max speed that the robot will move.

  chassis.turn(90, 100); //Turn to the right 90 degrees at 100 speed
  chassis.wait_drive();

  chassis.turn(-45, 75); //Turn to the left 45 degrees at 75 speed
  chassis.wait_drive();


  //If the first parameter is a Point or Pose, the robot will turn to face toward (or away from) the given point.

  chassis.turn(Point(70, 70), 100); //Turn to face the point (70, 70) at 100 speed
  chassis.wait_drive();

  chassis.turn(Point(-20, 30), 50, true); //Turn away from the point (-20, 30) at 50 speed
  chassis.wait_drive();
}


void arc_example() {
  //ALWAYS the robot's odom position based on where the physical robot is on the field! The center of the field is (0, 0).
  chassis.set_odom_position(10, 25, 90);


  //When using `.arc()`, the robot will turn to an angle by spinning only one side of the drive.
  //The first parameter for `.arc()` is the target angle relative to the robot (in degrees) to face.
  //The second parameter represents the direction that the robot will turn. So, a LEFT arc turns the robot left (counter clockwise) by spinning the right wheels.
  //The second parameter is the max speed that the robot will move.

  chassis.arc(45, Chassis::e_arc_direction::RIGHT, 100); //Arc to the right 45 degrees at 100 speed
  chassis.wait_drive();

  chassis.arc(135, Chassis::e_arc_direction::LEFT, 100); //Arc to the left 135 degrees at 100 speed
  chassis.wait_drive();
}


//Motion profiling is for advanced users only! It can be very powerful when used correctly, but it takes a lot of experience to use effectively.
void motion_profiling_example() {
  //ALWAYS the robot's odom position based on where the physical robot is on the field! The center of the field is (0, 0).
  chassis.set_odom_position(10, 25, 90);


  //When using `.motion_profiling()`, the robot will generate a path through the specified points and follow it using certain parameters.
  //If only one Point is in the `path`, the robot will drive smoothly to that point. If multiple Points are specified, the robot will attempt to reach all of them smoothly in the specified order.

  chassis.motion_profiling({Point(36, -12)}, -90, 1,0, 40, 25, -35, 10);
  chassis.wait_drive();

  chassis.motion_profiling({Point(24, 0)}, -45, 1, 0, 50, 25, -25, 10);
  chassis.wait_drive();
}

 void pid_test(){
 
 }

//This auton isn't actually needed, but can be helpful if your partner wants you to not run any auton, or if you need to run an auton as a test during inspection at the start of a competition.
void nothing() {}

void auton_test() {
  //void motion_profiling(std::vector<Point> path, double final_angle, double tangent_magnitude, double v1, double v_max, double a_accel, double a_decel, double w, bool reversed=false);
  chassis.set_odom_position(0,0,0);
  set_scoring(12000);
  chassis.motion_profiling({Point(-9,38.6)}, -45, 1.4,0, 60, 90, -100, 14); 
  pros::delay(855);
  tounge.set_value(true);
  chassis.wait_drive();
  
  chassis.turn(-130,100);
  chassis.wait_drive();
  
  chassis.drive(-7,100);
  chassis.wait_drive();
 
  holder.set_value(false);
   set_rollers(10000);
   pros::delay(200);
   set_rollers(12000);
  pros::delay(625);
  holder.set_value(true);
  set_scoring(12000);

  chassis.drive(44.8,100); //48.5
  chassis.wait_drive();
  
  chassis.turn(180,100);
  chassis.wait_drive();
  
  chassis.drive(16,70);
  chassis.wait_drive();
  pros::delay(800);
  roller_lift.set_value(true);
  descore_lift.set_value(true);
  chassis.drive(-17,70);
  chassis.wait_drive();
  chassis.set_odom_angle(180);
  chassis.turn(-183,100);
  chassis.wait_drive();
  chassis.drive(-15,100);
  chassis.wait_drive();
  //chassis.set_tank(-80,-80); //removed this line
  holder.set_value(false);

  //chassis.wait_drive();

}

void left_side_seven_ball(){

}