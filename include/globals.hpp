#pragma once

#include "globals.hpp"
#include "pros/adi.hpp"
#include "pros/apix.h"
#include "2616Lib/subsystems/flywheel.hpp"
#include "2616Lib/subsystems/catapult.hpp"
#include "pros/motors.h"

//Consolidated all the Port Assignements to make it easier to change ports

// ***************** Smart Ports *******************
//For motor ports, adding a - before the number causes it to be reversed
// ********* Chassis Motor Ports ************
#define LEFT_FRONT_PORT -19
#define LEFT_CENTER_PORT -20
#define LEFT_BACK_PORT 18
#define RIGHT_FRONT_PORT 15 
#define RIGHT_CENTER_PORT 16 
#define RIGHT_BACK_PORT -17

// ************* Sensor Ports *************
#define IMU_PORT_1 13
#define IMU_PORT_2 14
#define PERPENDICULAR_TRACKER_PORT 11
#define PARALLEL_TRACKER_PORT 12
#define PERP_DISTANCE_SENSOR_PORT 6

// ************* Others *************
#define INTAKE_PORT 3
#define TOP_ROLLER_PORT 1
#define THREE_WIRE_EXPANDER_PORT 2



// ************* 3 Wire Ports ************

// ************* Piston Ports *************
#define TOUNGE_PORT 'A'
#define DESCORE_PORT 'B'
#define BALL_HOLDER_PORT 'C'
#define DESCORE_LIFT_PORT 'D'
//this has same port because it goes into the 3 wire extendsion
#define ROLLER_LIFT_PORT 'D'
#define PISTON_ODOM_PORT 'H'

//************** Others *******************



//This area is used to initialize the objects everything that was made
//Chassis and tracking wheels are not included because it is done by the chassis class/constructor
//inline is used to prevent double definition

//*************** 3 Wire Extension **************
inline pros::ext_adi_port_pair_t ext_piston_odom(THREE_WIRE_EXPANDER_PORT, PISTON_ODOM_PORT);
inline pros::ext_adi_port_pair_t ext_tounge(THREE_WIRE_EXPANDER_PORT, TOUNGE_PORT);
inline pros::ext_adi_port_pair_t ext_descore(THREE_WIRE_EXPANDER_PORT,DESCORE_PORT);
inline pros::ext_adi_port_pair_t ext_descore_lift(THREE_WIRE_EXPANDER_PORT, DESCORE_LIFT_PORT);

// ************* Motor Objects *************
inline pros::Motor intake(INTAKE_PORT, pros::E_MOTOR_GEAR_600);
inline pros::Motor roller(TOP_ROLLER_PORT, pros::E_MOTOR_GEAR_600);


// ************* Sensor Objects *************
inline pros::Distance perp_sensor(PERP_DISTANCE_SENSOR_PORT);


// ************* Piston Objects *************
inline pros::ADIDigitalOut tounge(ext_tounge);
inline pros::ADIDigitalOut descore(ext_descore);
inline pros::ADIDigitalOut roller_lift(ROLLER_LIFT_PORT);
inline pros::ADIDigitalOut holder(BALL_HOLDER_PORT);
inline pros::ADIDigitalOut descore_lift(ext_descore_lift);
inline pros::ADIDigitalOut piston_odom(ext_piston_odom);


// ************* Sensor Objects *************
//These 2 are in there if you want to use limit switch to go between autons
 //inline pros::ADIDigitalIn auton_limit_switch_up('H');
 //inline pros::ADIDigitalIn auton_limit_switch_down('G');
