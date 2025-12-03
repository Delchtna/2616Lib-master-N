#include "main.h"
#include "2616Lib/subsystems/intake.hpp"
#include "2616Lib/util/util.hpp"
#include "autons.hpp"


/*********************************************************************************/
/*      This file is where you initialize the systems on your robot, set up      */
/*     your driver control code, and more. This is also where you set up your    */
/*           chassis object, which represents your robot's drive base.           */
/*********************************************************************************/


// ******************** Chassis Motor Ports ********************

//For more info about what these numbers mean, see globals.hpp
#define LEFT_FRONT_PORT -16
#define LEFT_CENTER_PORT -19
#define LEFT_BACK_PORT 20
#define RIGHT_FRONT_PORT 3
#define RIGHT_CENTER_PORT 12
#define RIGHT_BACK_PORT -6



/**********************************************************************/
/*                                                                    */
/*                        CHASSIS OBJECT SETUP                        */
/*                                                                    */
/**********************************************************************/

//The chassis is a single object that controls all the movement of your robot, and it also handles odometry if it's enabled
inline Chassis chassis (
  //Left motor ports
    { LEFT_FRONT_PORT, LEFT_CENTER_PORT, LEFT_BACK_PORT }
  //Right motor ports
  , { RIGHT_FRONT_PORT, RIGHT_CENTER_PORT, RIGHT_BACK_PORT }
  //IMU ports - if none, leave the list empty
  , { IMU_PORT_1, IMU_PORT_2 }


  /**********************************************************************/
  /*                                                                    */
  /*         THE REST OF THE CHASSIS OBJECT SETUP IS RELATED TO         */
  /*                    POSITION TRACKING (ODOMETRY).                   */
  /*                                                                    */
  /*                   If you are not using odometry,                   */
  /*         DO NOT change anything in the rest of this section.        */
  /*                                                                    */
  /**********************************************************************/

  //If you are using 2 parallel trackers and 1 perpendicular tracker, initialize a PERPENDICULAR, LEFT, and RIGHT Tracking_Wheel below.
  //If you are using 1 parallel tracker and 1 perpendicular tracker, initialize a PERPENDICULAR and *either* LEFT or RIGHT Tracking_Wheel below.
  //If you are using only 1 or 2 parallel trackers, initialize a LEFT and/or RIGHT Tracking_Wheel below.
  //If you aren't using any dedicated tracking wheels but still want odometry, initialize a LEFT and RIGHT Tracking_Wheel below using your front drive motors.
  //Regardless of your Tracking_Wheel configuration, make sure your IMUs are listed above!
  //Also, you can only define at most one Tracking_Wheel of each type (PERPENDICULAR, LEFT, or RIGHT)!

  //Tracking_Wheel setup:
    //Rotation sensors with dedicated tracking wheels are the most accurate, ADI encoders with dedicated tracking wheels are slightly less accurate, and motor encoders have the lowest accuracy but are the simplest to use.
    //The `wheel_diameter` parameter represents the size of the wheel connected to the sensor (in inches). This is either 2.75, 3.25, 4, or 4.125.
    //  Remember that the new anti-static 4" omni wheels are exactly 4", but the older (and much more common) ones are 4.125"
    //The `offset` parameter represents the distance (in inches) between the center of the wheel itself and the tracking center, which is an imaginary point that should be near the center of your robot.
    //  You will have to tune the `offset` parameter through real-life testing by turning the robot a known amount and comparing it to the angle tracked by the odometry.
    //The optional `ratio` parameter is the number of teeth on the sensor or motor gear divided by the number of teeth on the wheel gear. If the sensor or motor is directly connected to the wheel, leave this parameter as 1 or remove it.
    //  As an example, if your sensor or motor is geared 36:60 where the 36t is on the sensor or motor, the "ratio" would be 36 divided by 60, which is 0.6.

    //If you're using a rotation sensor, be sure to use the absolute value function `abs()` around the port number, and add the `reversed` boolean parameter after the sensor parameter rather than in the sensor constructor itself.
    //If you're using an ADI encoder, the top wire must be plugged into an odd numbered port ('A', 'C', 'E', or 'G') and the bottom wire must be plugged into the next highest port number.
    //  If your ADI encoder is plugged into a 3 wire expander instead of directly into the brain, use this parameter format for the sensor object:
    //    `pros::ADIEncoder(pros::ext_adi_port_tuple_t{ THREE_WIRE_EXPANDER_PORT, LEFT_TRACKER_PORT_TOP, LEFT_TRACKER_PORT_BOTTOM }, is_reversed)`
  
  //Below is an example of how to define each Tracking_Wheel type. Typically, your Tracking_Wheels will all use the same sensor type and wheel diameter. Dedicated tracking wheels typically use rotation sensors and 2.75" wheels.
  , {
      //Perpendicular tracking wheel
    Tracking_Wheel(Tracking_Wheel::e_tracker_type::PERPENDICULAR, pros::Rotation(abs(PERPENDICULAR_TRACKER_PORT)), true, 2, 7.3),
    Tracking_Wheel(Tracking_Wheel::e_tracker_type::LEFT, pros::Rotation(abs(PARALLEL_TRACKER_PORT)), false, 2, 1.25),

      //Left tracking wheel
    // , Tracking_Wheel(Tracking_Wheel::e_tracker_type::LEFT, pros::ADIEncoder(LEFT_TRACKER_PORT_TOP, LEFT_TRACKER_PORT_BOTTOM, false), 2.75, 3.5)
      //Right tracking wheel
    // , Tracking_Wheel(Tracking_Wheel::e_tracker_type::RIGHT, pros::Motor(RIGHT_FRONT_PORT, pros::E_MOTOR_GEAR_BLUE), 3.25, 4.5, 0.6)
  }

  //This parameter is optional. If you want to disable odometry entirely, set this to "true". Otherwise, leave it as "false" or comment it out.
  , false
);


//If your robot has a flywheel, uncomment the `Flywheel flywheel(...)` line below, the matching `extern Flywheel flywheel;` in globals.hpp under the `Subsystems` section, and the `flywheel.set_pidf_constants(...)` line in main.cpp in `initialize()`.
//The `motor_ports` parameter is a vector of port numbers for the motors that are powering the flywheel.
//The `ratio` parameter is the ratio of the number of teeth on the powered gear divided by the number of teeth on the flywheel's output gear.
//  As an example, if your flywheel is geared 12:60 where the 12t is powering the 60t, the `ratio` would be 12 divided by 60, which is 0.2.
//The `cartridge` parameter is optional and represents the motor cartridge used in all of your flywheel motors. This is normally a blue 600 RPM cartridge.
Flywheel flywheel({ -FLYWHEEL_PORT }, 0.25, pros::motor_gearset_e_t::E_MOTOR_GEAR_600);

//If your robot has a flywheel, uncomment BOTH this line and the matching line `extern Catapult catapult;` in globals.hpp under the `Subsystems` section.
//The `motor_ports` parameter is a vector of port numbers for the motors that are powering the catapult.
//The `limit_switch_port` parameter is the port letter of the limit switch that detects when the catapult is ready to shoot.
//The `automatic_button` parameter is the button that shoots the catapult and automatically resets it.
//The `manual_power_button` parameter is the button that powers the catapult only while it is pressed if the manual mode is enabled.
//The `manual_toggle_button` parameter is the button that toggles the manual mode. When enabled, the automatic control will not run. This is mainly used as a failsafe in case something breaks and the automatic catapult control isn't stopping correctly.
//The `cartridge` parameter is optional and represents the motor cartridge used in all of your catapult motors. This is normally a red 100 RPM cartridge.
// Catapult catapult({ CATAPULT_PORT }, CATAPULT_LIMIT_SWITCH_PORT, pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_B, pros::E_CONTROLLER_DIGITAL_X, pros::E_MOTOR_GEAR_100);


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  /************************************************************/
  /*               USER CONFIGURATION FUNCTIONS               */
  /************************************************************/
  
  //You should modify this section based on your preferences and needs. Not all of these are required, but they can be very useful if you want to use them. Each function has its own purpose and explanation.
  
  //This required function sets up the constants for all of your robot's PIDs and other movement systems. For more information, see the function definition in the autons.cpp file.
  set_default_movement_constants();
  
  //Normally, moving the joystick halfway means the robot goes half speed. With a joystick curve, moving the joystick halfway might move the robot at only 1/4 power. But, moving it all the way forward still moves the robot at full speed.
  //Using a joystick curve is better than slowing down your motors directly since that would prevent your motors from moving at full speed, which results in a lower top speed and torque.
  //This can give you finer control of your robot since more of the joystick's physical movement is dedicated to controlling the lower speeds.
  //The joystick curve is based on team 5225's code from 2018, and can be visualized using this graph: https://www.desmos.com/calculator/rcfjjg83zx. The x-axis is the joystick input, and the y-axis is the adjusted motor output.
  //The `scale` parameter corresponds to the intensity of the drive curve, where a higher number means more of the joystick's movement goes to low speed. This can give you finer control of the robot, but might make the robot feel less snappy.
  //  You can visualize different `scale` values by adjusting the `t` value on the graph. You can also check specific sets of joystick inputs and their respective motor outputs by clicking on points on the graph.
  //The `curve_type` parameter must be exactly either "red" or "blue". This corresponds to the colors of the lines on the graph, and red is the default.
  //To disable the joystick curve entirely, set the `scale` to 0.
  chassis.set_joystick_curve(5, "red");

  //If you set your drive motors to the `hold` brake mode, your robot can still be pushed around by other teams. Active brake runs a P loop on the drive when you let go of the joysticks, which makes the robot actively resist external movement.
  //The `kP` parameter represents the strength of the active braking, where a higher value means stronger braking. If `kP` is too low, the robot will get pushed a bit before fighting back. If 'kP` is too high, the robot will oscillate or move jerkily back to its original position.
  //The `threshold` parameter determines when you have let go of the joysticks to enable active braking. A higher value means you need to move a joystick further to stop braking, and a lower value means the joystick needs to be closer to the center before braking will enable.
  //To disable active braking entirely, set `kP` to 0.
  chassis.set_active_brake_power(0, 10);


  //This function is only relevant if your robot uses IMUs, AND your odometry configuration uses EITHER a perpendicular, left, and right tracker, OR a left and right tracker only. Otherwise, you can ignore this function entirely.
  //The robot's odometry angle can be tracked using either output from the IMUs or by doing calculations using movement of the the tracking wheels. **USING THE IMUs IS GENERALLY MORE ACCURATE**, but the odometry configurations listed above support both methods.
  //So, if for some reason you want to track the robot's angle using the left and right tracking wheels instead of the IMUs, set the parameter to `true`. If your odometry configuration does not support this, IMUs will be used instead.
  //The parameter being `true` means the odom angle should be tracked using the left and right tracking wheels (if possible), and `false` means the odom angle should be tracked using IMUs (if possible).
  //Even if you leave this parameter as `false` to prefer using the IMUs, the code will try to use the wheel calculation method if the IMUs are throwing errors.
  chassis.prefer_wheel_calculated_odom_angle(false);
  
  //If your robot does not have a flywheel, comment out this line. Also, see the comments above and in globals.hpp for more info about which lines need to be uncommented if you're using a flywheel.
  //The first 5 parameters of this function are used to set the PIDF constants for the flywheel. For more info about PIDs, see the comments at the top of autons.cpp. The `f` term is for feedforward, which adds the term `target * kF` to the output.
  //The `pidf_threshold` parameter determines when the flywheel's PIDF should run. When the difference between the actual and target speed (aka error) is outside of this threshold, it runs a Bang Bang controller instead.
  //When the error is positive (meaning the flywheel needs to speed up) and is greater than the threshold, the motor runs at full speed to speed up quickly.
  //When the error is negative (meaning the flywheel needs to slow down) and is less than the threshold * -1, the motor coasts to slow down without damaging the motor.
  //Otherwise, the error is between +/- threshold, the PIDF controller runs to accurately reach the target speed.
  flywheel.set_constants(10.5, 5, 0, 50, 4, 200);

  
  //If your robot uses autons, the Auton Selector allows you to choose which auton routine you want to run in real time while the brain is turned on and your program is running.
  //Without an auton selector, you would have to upload each auton routine to a different program slot on your brain, which is difficult to maintain if you make any changes to your code or need to quickly change which routine to run during a match.
  //This auton selector allows you to load all your auton routines into one program slot on your brain, which is almost always preferrable since it makes it easier to maintain, and since it allows you to have more than 8 autons loaded at once.
  //If you don't have any autons, keep the `init({...})` method, but leave the `autons` vector empty.
  //If you have autons declared in autons.hpp and defined in autons.cpp (see autons.cpp for more details about this), use the format below to initiailze the auton selector. Each auton routine should have exactly one matching `Auton` object in the vector.
  //An `Auton` object contains a `name`, a `description`, and an `auton_call`. The `name` parameter can be whatever you want, but make sure it isn't too long so that it fits on the brain screen.
  //The `description` parameter can be whatever you want, including an empty string. If your description is too long to fit on one line of the brain screen, you can add AT MOST one newline character `\n` to split the description onto two lines.
  //The `auton_call` parameter is a call to the name of the function you want to run during autonomous, and SHOULD NOT run the function itself. For example, `my_auton` is correct, but `my_auton()` will not work correctly.
  //To change the selected auton, you can tap on either of the colored rectangle buttons on the bottom left corner of the brain screen when the program is running. The selected auton will be printed on both the brain screen and on your controller.
  Auton_Selector::init({
    //Auton_Selector::Auton("Drive example", "Test the .drive(...) method", drive_example),
    //Auton_Selector::Auton("Testing PID", "TO tune the PID method", pid_test),
    Auton_Selector::Auton("Auton Testing", "To create Autons", auton_test),
    //Auton_Selector::Auton("Nothing", "Don't do anything! \n:)", nothing)
  });
  
  //Comment this out if you aren't using limit switch(es) to select your auton.
  //If you want to be able to change the selected auton without needing to tap on the brain screen directly, you can also use one or two limit switches to control it too.
  //The `up_limit` parameter must be a reference to an existing limit switch defined in globals.hpp, and it INCREASES the page number by one each time it is pressed.
  //The optional `down_limit` parameter must also be a reference to an existing limit switch defined in globals.hpp, and it DECREASES the page number by one each time it is pressed.
  //If you only have one limit switch, you will still be able to cycle through all your autons, but you won't be able to move the page backward. However, the brain screen buttons will still work if you need to do this.
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
  //DO NOT CHANGE THESE LINES!
  chassis.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  chassis.set_drive_mode(Chassis::e_drive_mode::STANDBY);
  Auton_Selector::disable_auton_selector();

  while (true) {
    //todo add docs about what can be changed here

    chassis.tank_drive();
    // chassis.arcade_drive();
    control_rollers();
    bool intake_state = false;
    bool tounge_state = false;
    bool redirect_state = false;
    control_raise_intake();
    control_tounge();
    control_redirect();

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
