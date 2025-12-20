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
  chassis.drive_PID.set_constants(13, 0, 75, 0);
  chassis.turn_PID.set_constants(105, 0, 850, 0);
  chassis.heading_PID.set_constants(120, 0, 0, Util::to_rad(2.5));
  chassis.arc_PID.set_constants(250.0, 35, 85, Util::to_rad(2.5));
  chassis.set_bound(2);

  //PID exit conditions
  //These determine when a PID should stop running. If the PID's error is less than `small_error` (in either inches or radians) for at least `exit_time` milliseconds, the PID will stop running.
  chassis.drive_PID.set_exit_conditions(1.5, 300, 5, 1000, 0.01, 500);
  chassis.turn_PID.set_exit_conditions(Util::to_rad(2.5), 250, Util::to_rad(7), 400, Util::to_rad(1) * 0.01, 500);
  chassis.arc_PID.set_exit_conditions(Util::to_rad(2), 500, Util::to_rad(5), 500, Util::to_rad(1) * 0.01, 500);

  //Motion profiling - ADVANCED USERS ONLY
  chassis.path_traverser.set_software_constants(13, 1.834, 0.28, 7.55, 0.7, 12, 0, 10, 10, 5);
  chassis.path_traverser.set_hardware_constants(1.5, 12.8, 4.22);
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
  
  chassis.set_odom_position(0,0,0);
  chassis.arc(-135,Chassis::e_arc_direction::RIGHT, 100);
 }

//This auton isn't actually needed, but can be helpful if your partner wants you to not run any auton, or if you need to run an auton as a test during inspection at the start of a competition.
void nothing() {}

void auton_test() {
  
}



void top_and_mid(){
   chassis.set_odom_position(0,0,-12.3);
  redirect.set_value(true);
  raise_intake.set_value(true);
  set_rollers(12000);
  chassis.drive(33, 100);
  chassis.wait_drive();
  chassis.turn(57,120);
  tounge.set_value(true);
  chassis.wait_drive();
  set_rollers(0);
  redirect.set_value(false);
  chassis.drive(8.5, 120);
  pros::delay(200);
  tounge.set_value(false);
  chassis.wait_drive();
  score_mid(12000);
  pros::delay(1000);
  tounge.set_value(true);
  set_rollers(0);

  redirect.set_value(true);
  chassis.turn(-135,120);
  chassis.wait_drive();
  chassis.drive(40, 120);
  chassis.wait_drive();
  set_rollers(12000);
  chassis.turn(177,120);
  chassis.wait_drive();

  chassis.drive(16,90);
  chassis.wait_drive();
  pros::delay(600);

  chassis.drive(-10,120);
  tounge.set_value(false);
  chassis.wait_drive();
  
  chassis.turn(-5,120);
  
  chassis.wait_drive();
  chassis.drive(14, 90);
  redirect.set_value(false); 

  chassis.wait_drive();

}
void top_and_bottom(){
 chassis.set_odom_position(0,0,12.3);
  redirect.set_value(true);
  set_rollers(13000);
  chassis.drive(30, 100);
  chassis.wait_drive();
  chassis.turn(-48,120);
  chassis.wait_drive();
  redirect.set_value(false);
  chassis.drive(9, 120);
  set_rollers(0);
  pros::delay(200);
  chassis.wait_drive();
  score_bottom(12000);
  pros::delay(2000);
  set_rollers(0);

  redirect.set_value(true);
  
  chassis.drive(-48, 120);
  tounge.set_value(true);
  chassis.wait_drive();
  set_rollers(12000);
  chassis.turn(-177,120);
  chassis.wait_drive();

  chassis.drive(16,85);
  chassis.wait_drive();
  pros::delay(600);
 
  chassis.drive(-10,120);
  tounge.set_value(false);
  chassis.wait_drive();
  
  chassis.turn(0,120);
  
  chassis.wait_drive();
  chassis.drive(14, 90);
  redirect.set_value(false); 

  chassis.wait_drive();

}
void left_top(){
  chassis.set_odom_position(0,0,-12.3);
  redirect.set_value(true);
  raise_intake.set_value(true);
  set_rollers(12000);
  chassis.drive(33, 80);
  chassis.wait_drive();
  chassis.turn(-140,100);
  chassis.wait_drive();
  chassis.drive(36,100);
  chassis.wait_drive();
  chassis.turn(174,100); //this rn
  chassis.wait_drive();
  tounge.set_value(true);
   chassis.drive(13.5,100);
  chassis.wait_drive();
  pros::delay(800);
  chassis.drive(-10,100);
  tounge.set_value(false);
  chassis.wait_drive();
  chassis.turn(3,100);
  chassis.wait_drive();
  chassis.drive(15, 90);
  chassis.wait_drive();

  redirect.set_value(false);
}
void right_top(){
  chassis.set_odom_position(0,0,-12.3);
  redirect.set_value(true);
  raise_intake.set_value(true);
  set_rollers(12000);
  chassis.drive(33, 80);
  chassis.wait_drive();
  chassis.turn(140,100);
  chassis.wait_drive();
  chassis.drive(36,100);
  chassis.wait_drive();
  chassis.turn(-174,100); //this rn
  chassis.wait_drive();
  tounge.set_value(true);
   chassis.drive(13.5,100);
  chassis.wait_drive();
  pros::delay(800);
  chassis.drive(-10,100);
  tounge.set_value(false);
  chassis.wait_drive();
  chassis.turn(-3,100);
  chassis.wait_drive();
  chassis.drive(15, 90);
  chassis.wait_drive();

  redirect.set_value(false);
}
//Add more autons here!