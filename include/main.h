/**********************************************************************************/
/*            This file is only used for including other header files,            */
/*           so you can include the entire library into your .cpp files           */
/*                 using the following line at the top of a file:                 */
/*                                #include "main.h"                               */
/*                                                                                */
/*    To create variables that will be used in multiple files, see globals.hpp    */
/**********************************************************************************/

/**
 * \file main.h
 *
 * Copyright (c) 2017-2022, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_


/*****************************************************************************/
/*                                                                           */
/*              Include all of your files in the section below.              */
/*              Whenever you create a new file, include it here.             */
/*                                                                           */
/*****************************************************************************/


#include "2616Lib/auton/auton_selector.hpp"

#include "2616Lib/chassis/chassis.hpp"
#include "2616Lib/chassis/motion/motion_profiling.hpp"
#include "2616Lib/chassis/motion/pure_pursuit.hpp"
#include "2616Lib/chassis/odometry/tracking_wheel.hpp"

#include "2616Lib/path_generation/path_generation.hpp"
#include "2616Lib/path_generation/quintic_bezier.hpp"

#include "2616Lib/subsystems/catapult.hpp"
#include "2616Lib/subsystems/flywheel.hpp"
#include "2616Lib/subsystems/indexer.hpp"
#include "2616Lib/subsystems/intake.hpp"
#include "2616Lib/subsystems/pistons.hpp"

#include "2616Lib/util/points/path_point.hpp"
#include "2616Lib/util/points/point.hpp"
#include "2616Lib/util/points/pose.hpp"
#include "2616Lib/util/PID.hpp"
#include "2616Lib/util/PIDF.hpp"
#include "2616Lib/util/simple_moving_average.hpp"
#include "2616Lib/util/util.hpp"








/**********************************************************************/
/*               DO NOT CHANGE ANYTHING BELOW THIS LINE               */
/**********************************************************************/



//"pros/apix" is included in globals.hpp
#include "globals.hpp"
#include "autons.hpp"

#include <string>
#include <vector>
#include <mutex>
#include <algorithm>
#include <atomic>
#include <utility>

extern Chassis chassis;
inline pros::Mutex mutex;
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);


/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */

//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
