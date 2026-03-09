#include "2616Lib/chassis/chassis.hpp"
#include "2616Lib/subsystems/intake.hpp"
#include "2616Lib/subsystems/pistons.hpp"
#include "2616Lib/util/util.hpp"
#include "globals.hpp"
#include "main.h"
#include "2616Lib/subsystems/sensor.hpp"
#include "pros/motors.h"

// ************* Setting Movement Constants *************

void set_default_movement_constants() {
  //PID constants
  //Your robot uses PIDs to control the drive motors during autonomous to smoothly move from a starting position to a target. They are used for driving straight, turning, and more.
  //For more information on how PID controllers work, reference this document made by George Gillard:   https://georgegillard.com/resources/documents.
  //EZ-Template has a good tutorial for tuning PIDs, which can be found here:   https://ez-robotics.github.io/EZ-Template/Tutorials/tuning_constants#tuning-pid.
  //On EZ Template's website, IGNORE THE CODE ITSELF, since 2616Lib's code isn't compatible with EZ's. However, the information about tuning a PID is still very useful. Also, 2616Lib doesn't use slew, so you can ignore that section on EZ's website.
  chassis.drive_PID.set_constants(10, 0,30, 0); //10, 0, 43
  chassis.turn_PID.set_constants(260.0, 35, 1200, Util::to_rad(2.5));
  chassis.heading_PID.set_constants(120, 0, 0, Util::to_rad(2.5));
  chassis.arc_PID.set_constants(250.0, 35, 85, Util::to_rad(2.5));
  chassis.set_bound(2);

  //PID exit conditions
  //These determine when a PID should stop running. If the PID's error is less than `small_error` (in either inches or radians) for at least `exit_time` milliseconds, the PID will stop running.
  chassis.drive_PID.set_exit_conditions(5, 700, 10, 1500, 0.01, 500);
  chassis.turn_PID.set_exit_conditions(Util::to_rad(2), 250, Util::to_rad(5), 500, Util::to_rad(1) * 0.01, 500);
  chassis.arc_PID.set_exit_conditions(Util::to_rad(2), 500, Util::to_rad(5), 500, Util::to_rad(1) * 0.01, 500);

  //Motion profiling - ADVANCED USERS ONLY
  chassis.path_traverser.set_software_constants(.19, 1.6171, 1.3493, .33 , 8.3,  .7, 6 , .33,0, 10, 10, 5);  
  chassis.path_traverser.set_hardware_constants(2.25, 11, 3.25);
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
  //set_intake(120);
  
  //chassis.wait_drive();
  //chassis.turn_test(5000);
  //chassis.turn(0,50);
  //chassis.wait_drive();
  //std::cout<<""<<chassis.get_pose().x<<","<<chassis.get_pose().y<<","<<chassis.get_pose().angle;
  //chassis.set_odom_position(0,0,0);
  //chassis.turn(180,50);
//ka = .183
//kP_theta .68-.71
//kp = 2.98
//beta = 4
//try this one before changing 
  //chassis.path_traverser.set_software_constants(.19, 1.6171, 1.3493, .33 , 8.3,  .7, 5 , .315,0, 10, 10, 5);
  //chassis.motion_profiling({Point(24,55)},90,1,0, 30, 40, -40, 9.5); 
 //chassis.ramp_voltage_test(.5);
  //chassis.wait_drive();
  //chassis.drive(Point(24,32),100);
  //chassis.motion_profiling({Point(48,0)}, 180,1,0, 25, 35, -40, 14);
  //chassis.set_tank(80,0);
  //chassis.drive(24,80);
 }

//This auton isn't actually needed, but can be helpful if your partner wants you to not run any auton, or if you need to run an auton as a test during inspection at the start of a competition.
void nothing() {}

void auton_test() { 
  //SKILLS
/*
  
  //Section 1: RED PARK CLEAR
  chassis.set_odom_position(0,0,90); 

  piston_odom.set_value(true); //piston odom up
  
  set_scoring(12000);
  
  chassis.drive(13, 75);
  pros::delay(540);
  tounge.set_value(true);//tongue down
  chassis.wait_drive();
  
  chassis.drive(33,80);
  tounge.set_value(false);
  pros::delay(300);
  tounge.set_value(true);
  chassis.wait_drive();
  
  chassis.turn(0,90);
  chassis.wait_drive();
  
  tounge.set_value(false);
  
  chassis.set_tank(-80,-80);
  pros::delay(200);
  chassis.set_tank(0,0);
  
  piston_odom.set_value(false);
  
*/
  //Section 2: BOTTOM GOAL
  
  chassis.set_odom_position(-62,get_perp_dist(),90);
  
  
  
  chassis.turn(87,100);
  pros::delay(800);
   set_intake(12000);
  chassis.wait_drive();
  
  chassis.drive(47,100);
  pros::delay(520);
  tounge.set_value(true);
  chassis.wait_drive();
 
  tounge.set_value(false);

  chassis.drive(-11.75,100);
  set_intake(0);
  chassis.wait_drive();

  chassis.turn(48,100);
  chassis.wait_drive();
  
  chassis.drive(14,120);
  chassis.wait_drive();
 
  set_scoring(-10000,-12000);
  pros::delay(495);
  set_scoring(-11000,-10000);
  pros::delay(300);
  set_scoring(-8000);
  pros::delay(1450);
  
  chassis.motion_profiling({ Point(-23.3, -14.9), Point(-24, 0)},180, 1.2, 0, 30, 25, 30, 3.142, true);
  chassis.wait_drive();

  chassis.turn(0,100);
  chassis.wait_drive();
  
  set_scoring(12000);
  
  chassis.motion_profiling({ Point(-24, 24.1),Point(-48, 45)},0, 1.4, 0, 40, 60, -70, 3.142, false);
  pros::delay(400);
  tounge.set_value(true);
  chassis.wait_drive();
  
  Pose set = chassis.get_pose();
  pros::delay(100);
  roller_lift.set_value(true);
  pros::delay(100);
  chassis.set_odom_position(set);

  chassis.turn(-80,100);
  chassis.wait_drive();

  chassis.drive(-24,100);
  chassis.wait_drive();

  holder.set_value(false);
  pros::delay(500);
  holder.set_value(true);
  
/*
  //SECTION 3: RED TOP MATCHLOAD 
  chassis.set_odom_position(-34.5,48,-90);
  
  tounge.set_value(true);
  
  set_scoring(12000);
  
  chassis.turn(Point(-68,48), 100);
  chassis.wait_drive();
  
  chassis.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  
  chassis.drive(28.7, 70);
  chassis.wait_drive();
  
  pros::delay(1500);
  
  chassis.drive(-4,100);
  chassis.wait_drive();
  
  chassis.motion_profiling({ Point(-24, 33.5),Point(0,33.5)}, -85,1.5, 0, 40, 50, -60, 3.142, true); 
  chassis.wait_drive();
  
  chassis.drive(-24,100);
  chassis.wait_drive();
  
  chassis.motion_profiling({  Point(47.7, 48)},-135, 1.9, 0, 40, 50, -60, 3.142, true);
  chassis.wait_drive();
  
  chassis.turn(94, 100);
  chassis.wait_drive();
  
  chassis.drive(-20, 100);
  chassis.wait_drive();
  
  holder.set_value(false);
  pros::delay(1000);
  

  //SECTION 4: BLUE TOP MATCHLOAD
  chassis.set_odom_position(34.5,48,90);
  
  set_scoring(12000);
  
  chassis.turn(93,100);
  chassis.wait_drive();
  
  chassis.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  
  chassis.motion_profiling({  Point(67.55, 47)},90, .3, 0, 30, 50, -60, 3.142, false);
  chassis.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  chassis.wait_drive();

  pros::delay(1500);

  chassis.motion_profiling({  Point(34.5, 48.5)},90, .3, 0, 20, 40, -50, 3.142, true);
  chassis.wait_drive();
  
  holder.set_value(false);
  
  chassis.set_tank(0,-80);
  pros::delay(1250);
  chassis.set_tank(0,0);
  holder.set_value(true); 

  //SECTION 5: BLUE PARK
  chassis.set_odom_position(34.5, 48, 90);
  
  descore_lift.set_value(true);
  
  tounge.set_value(false);

  chassis.motion_profiling({ Point(55, 39), Point(65.6, 20),Point(66,12)},180, 1.1, 0, 45, 60, -65, 3.142, false);  chassis.wait_drive();
  roller_lift.set_value(false);
  pros::delay(500);
  chassis.wait_drive();

  piston_odom.set_value(true); //piston odom up
  
  set_scoring(12000);
  
  chassis.drive(15, 95);
  pros::delay(1100);
  tounge.set_value(true);//tongue down
  chassis.wait_drive();
  
  chassis.drive(27,90);
  tounge.set_value(false);
  pros::delay(800);
  tounge.set_value(true);
  chassis.wait_drive();

  chassis.turn(-90, 100);
  chassis.wait_drive();
  
  chassis.set_tank(-80,-80);
  pros::delay(200);
  chassis.set_tank(0,0);
  
  piston_odom.set_value(false);


  //SECTION 6: TOP MID GOAL
  chassis.set_odom_position(62,get_perp_dist(),-90);
  //chassis.set_odom_position(60.5,-24,-90);
  
  tounge.set_value(false);
  
  set_scoring(12000);
  
  holder.set_value(true);
  
  chassis.path_traverser.set_software_constants(.19, 1.6171, 1.3493, .33 , 8.3,  .7, 6 , .33,0, 10, 10, 5);  
  chassis.set_bound(4);
  chassis.motion_profiling({ Point(24, -24)},-139, 1.5, 0, 50, 60, -70, 3.142, false);
  pros::delay(1500); 
  tounge.set_value(true);
  chassis.wait_drive();
  
  set_scoring(0);
  
  chassis.drive(3, 100);
  chassis.wait_drive();
  
  chassis.turn(Point(8.5, -8.5), 100, true);
  chassis.wait_drive();
  
  chassis.drive(-10.5, 100);
  chassis.wait_drive();
  
  set_scoring(0);
  chassis.wait_drive();
  
  holder.set_value(false);
  
  set_scoring(9500);
  pros::delay(2800);
  set_scoring(0);
  
  holder.set_value(true);
  
  //Long Goal Scoring
  chassis.drive(42.5, 120);
  roller_lift.set_value(true);
  chassis.wait_drive();
  
  chassis.turn(90, 100);
  chassis.wait_drive();
  
  chassis.drive(-15, 120);
  chassis.wait_drive();
  
  holder.set_value(false);
  
  set_scoring(12000);
  
  pros::delay(500);
 

  //SECTION 7: BLUE BOTTOM MATCHLOAD
  chassis.set_odom_position(24,-48,90);
  
  tounge.set_value(true);
  
  set_scoring(12000);
  
  chassis.drive(27.75,60);
  chassis.wait_drive();
  
  pros::delay(1000);
  
  chassis.drive(1.5,120);
  chassis.wait_drive(  );
  
  pros::delay(500);
  
  chassis.path_traverser.set_software_constants(.19, 1.6171, 1.3493, .33 , 8.3,  .7, 6 , .33,0, 10, 10, 5);  
  chassis.set_bound(5);
  chassis.motion_profiling({ Point(32., -32), Point(-40, -32), Point(-48, -44)}, 45, 1.1, 0, 50, 30, -60, 3.142, true);// //Tuning Turn PID Over Small Turns  
  chassis.wait_drive();

  chassis.turn(-90,100);
  chassis.wait_drive();
  
  chassis.drive(-10,100);
  chassis.wait_drive();

  holder.set_value(false);


  //SECTION 7: RED BOTTOM MATCHLOAD
  chassis.set_odom_position(-24,-48,-90);
  
  set_scoring(12000);
  
  chassis.drive(27.75,60);
  chassis.wait_drive();
  
  pros::delay(1000);
  
  chassis.drive(1.5,120);
  chassis.wait_drive(  );
  
  pros::delay(500);
  
  chassis.turn(Point(-24,-48),100,true);
  chassis.wait_drive();

  chassis.drive(-27,100);
  chassis.wait_drive();
  
  chassis.set_tank(0,-80);
  holder.set_value(false);
  pros::delay(10000);
  chassis.set_tank(0,0);
  
  
  //SECTION 8: PARK
  chassis.set_odom_position(-34,-48,-90);
  
  tounge.set_value(false);
  
  chassis.motion_profiling({ Point(-64, -19.5)}, 0, 1.1, 0, 50, 60, 60, 3.142, false);
  chassis.wait_drive();
  
  piston_odom.set_value(true);
  
  chassis.drive(17,100);
  chassis.wait_drive();
  */
}

void left_side_split(){
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
  Pose set = chassis.get_pose();
  pros::delay(100);
  roller_lift.set_value(true);
  descore_lift.set_value(true);
  pros::delay(100);
  chassis.set_odom_position(set);
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