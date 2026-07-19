#include "main.h"
#include "2616Lib/chassis/chassis.hpp"
#include "2616Lib/chassis/motion/motion_profiling.hpp"
#include "2616Lib/subsystems/intake.hpp"
#include "2616Lib/util/util.hpp"
#include "autons.hpp"
#include "globals.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <fstream>
#include "2616Lib/subsystems/pistons.hpp"
#include "2616Lib/subsystems/sensor.hpp"


/*********************************************************************************/
/*      This file is where you initialize the systems on your robot, set up      */
/*     your driver control code, and more. This is also where you set up your    */
/*           chassis object, which represents your robot's drive base.           */
/*********************************************************************************/


// ******************** Chassis Motor Ports ********************

//For more info about what these numbers mean, see globals.hpp
#define LEFT_FRONT_PORT -19
#define LEFT_CENTER_PORT -20
#define LEFT_BACK_PORT 18
#define RIGHT_FRONT_PORT 15 
#define RIGHT_CENTER_PORT 16 
#define RIGHT_BACK_PORT -17

/**********************************************************************/
/*                                                                    */
/*                        CHASSIS OBJECT SETUP                        */
/*                                                                    */
/**********************************************************************/

//The chassis is a single object that controls all the movement of your robot, and it also handles odometry if it's enabled

inline Chassis chassis (
    { LEFT_FRONT_PORT, LEFT_CENTER_PORT, LEFT_BACK_PORT }, 
    { RIGHT_FRONT_PORT, RIGHT_CENTER_PORT, RIGHT_BACK_PORT }, 
    { IMU_PORT_1, IMU_PORT_2 },

/**************************************************************************/
/*                                                                        */
/*           THE REST OF THE CHASSIS OBJECT SETUP IS RELATED TO           */
/*                       TRACKING WHEELS(ODOMETRY).                       */
/*                                                                        */
/*   It is HIGHLY RECOMMENDED that you use odometry, but if you decide    */
/*         not to, DO NOT change anything in the rest of this section.    */
/*                                                                        */
/**************************************************************************/

  //Perpendicular wheels is there to get the left and right slip while moving
  //Left and Right wheels used to be used together to get angle, but IMU is preferred, even tho IMU can drift
  //Either LEFT or RIGHT is used to get movement foward and backwards  
  //Motor option for those that want odometry, but don't want tracking wheels
  //abs() and reversed parameters needed for rotational sensors due to a glitch on pros where initializing rotational sensor with - leads to error
  //gear ratio is for the motors, but available to all
  
  {
    Tracking_Wheel(Tracking_Wheel::e_tracker_type::PERPENDICULAR, pros::Rotation(abs(PERPENDICULAR_TRACKER_PORT)), false, 2, 6.65),
    Tracking_Wheel(Tracking_Wheel::e_tracker_type::LEFT, pros::Rotation(abs(PARALLEL_TRACKER_PORT)), false, 2,.21),
  },

   false
);




  
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
 
  holder.set_value(true);
  tounge.set_value(false);
  
  /************************************************************/
  /*               USER CONFIGURATION FUNCTIONS               */
  /************************************************************/
  
  //This is here so that the robot does not start without the robot setting the movement constants 
  set_default_movement_constants();
  
  //This function exists to give more control to the user by setting more joystick movement to slow
  //This is better than setting the motor to lower speed as that would result in less torque and less speed, and the motor would never get to full speed
  chassis.set_joystick_curve(5, "red");

  //Exists to fight back against change in velocity by outside forces
  //If kD is too low it won't do anything when pushed back and just slide
  //If kD is too high it will overreact to small forces. 
  //kD as 0 to disable is for in case this doesn't work
  chassis.set_active_brake_power(0, 10);
  
  //If your robot uses autons, the Auton Selector allows you to choose which auton routine you want to run in real time while the brain is turned on and your program is running.
  //Without an auton selector, you would have to upload each auton routine to a different program slot on your brain, which is difficult to maintain if you make any changes to your code or need to quickly change which routinett to run during a match.
  //This auton selector allows you to load all your auton routines into one program slot on your brain, which is almost always preferrable since it makes it easier to maintain, and since it allows you to have more than 8 autons loaded at once.
  //If you don't have any autons, keep the `init({...})` method, but leave the `autons` vector empty.
  //If you have autons declared in autons.hpp and defined in autons.cpp (see autons.cpp for more details about this), use the format below to initiailze the auton selector. Each auton routine should have exactly one matching `Auton` object in the vector.
  //An `Auton` object contains a `name`, a `description`, and an `auton_call`. The `name` parameter can be whatever you want, but make sure it isn't too long so that it fits on the brain screen.
  //The `description` parameter can be whatever you want, including an empty string. If your description is too long to fit on one line of the brain screen, you can add AT MOST one newline character `\n` to split the description onto two lines.
  //The `auton_call` parameter is a call to the name of the function you want to run during autonomous, and SHOULD NOT run the function itself. For example, `my_auton` is correct, but `my_auton()` will not work correctly.
  //To change the selected auton, you can tap on either of the colored rectangle buttons on the bottom left corner of the brain screen when the program is running. The selected auton will be printed on both the brain screen and on your controller.
  
  //Creates an Auton Selector object
  //This means you don't have to upload each auton to a different slot
  //You can load all your autons into one slot and choose before the match starts
  Auton_Selector::init({
    //Auton_Selector::Auton("Drive example", "Test the .drive(...) method", drive_example),
    //Auton_Selector::Auton("Testing PID", "TO tune the PID method", pid_test)
  Auton_Selector::Auton("Auton Testing", "To create Autons", auton_test)
    //Auton_Selector::Auton("Nothing", "Don't do anything! \n:)", nothing)
    //Auton_Selector::Auton("Top and Mid", "Scoring on Top and Middle! \n:)", top_and_mid),
    //Auton_Selector::Auton("Top and Bottom", "Scoring on Top and Bottom! \n:)", top_and_bottom),
    //Auton_Selector::Auton("Left Top", "Scoring all on Left! \n:)", left_top),
    //Auton_Selector::Auton("Right Top", "Scoring all on Right! \n:)", right_top)
  });
  
  //Comment this out if you aren't using limit switch(es) to select your auton.
  //If you want to be able to change the selected auton without needing to tap on the brain screen directly, you can also use one or two limit switches to control it too.
  //The `up_limit` parameter must be a reference to an existing limit switch defined in globals.hpp, and it INCREASES the page number by one each time it is pressed.ed.
  //If you only have one limit switch, you will still be able to cycle through all your autons, but you won't be able to move the page backward. However, the brain screen buttons
  //The optional `down_limit` parameter must also be a reference to an existing limit switch defined in globals.hpp, and it DECREASES the page number by one each time it is press will still work if you need to do this.
  //Make sure to define your limit switch(es) correctly in globals.hpp! Passing a local variable (or a limit switch initialized with the wrong port) to this method will break the entire auton selector!
  // Auton_Selector::limit_switch_initialize(&auton_limit_switch_up, &auton_limit_switch_down);

  //DO NOT CHANGE THIS LINE!
  chassis.start_tasks();
  

  
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}


/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

 //DO NOT CHANGE ANYTHING IN THIS FUNCTION!
void autonomous() {
  chassis.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  Auton_Selector::run_selected_auton();
}



  
  
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator 
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  int ballcount = 0;
  //DO NOT CHANGE THESE LINES!
  chassis.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  chassis.set_drive_mode(Chassis::e_drive_mode::STANDBY);
  Auton_Selector::disable_auton_selector();

  while (true) {
    //todo add docs about what can be changed here
    
    chassis.tank_drive();
    // chassis.arcade_drive();
    control_intake();
    
    control_holder();
    control_descore();
    control_lift();
    control_tounge();
  
    /*
    // L1 --> forward, L2 --> backward, intake
		endgameExpansion(); // X and Up
		flywheel.setFlywheelMotors(); // Up, Down, Left, Right, A, X, Y
		indexerControl(); // R1 // R2
		angleChange(); // B

    setSubsystemMotors();
    */

    //DO NOT CHANGE THIS LINE!
    pros::delay(Util::DELAY_TIME);
  }
}
