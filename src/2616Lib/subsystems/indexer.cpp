#include "main.h"


/*************************************************************/
/*                                                           */
/*               A quick note about indexers:                */
/*                                                           */
/*  Your robot might have an indexer with its own dedicated  */
/*     motor, an indexer connected to your intake's motor,   */
/*    an indexer using one or more pistons, or no indexer    */
/*       at all. So, you should modify this subsystem        */
/*              based on what your robot uses.               */
/*                                                           */
/*    This subsystem comes with some methods for indexers    */
/*       using a dedicated motor and a piston, and the       */
/*     relevant PROS objects are defined in globals.hpp.     */
/*     So, you should comment out (or delete) the methods    */
/*       and PROS objects that your robot doesn't need.      */
/*                                                           */
/*************************************************************/


//Spin the indexer motor at a certain voltage. Voltage range is [-12000, 12000], and 0 stops the motor.
// void set_indexer(int voltage) {
//   if (voltage == 0) {
//     indexer_motor.move_velocity(0);
//   } else {
//     indexer_motor.move_voltage(voltage);
//   }
// }

// bool indexer_state = false;
// //Set the state of the indexer piston to a new value
// void set_indexer_piston(bool state) {
//   indexer_piston.set_value(state);
//   indexer_state = state;
// }

// //Toggle the state of the indexer piston
// void toggle_indexer() {
//   set_indexer_piston(!indexer_state);
// }


// //Use this method if you have a motor indexer
// void shoot(int num, int shooting_duration, int time_between_shots) {
//   for (int i = 0; i < num; i++) {
//     //Use this if your indexer uses a dedicated motor
//     set_indexer(12000);
//     pros::delay(shooting_duration);
//     set_indexer(0);

//     //Use this if your indexer uses a piston
//     // set_indexer_piston(true);
//     // pros::delay(shooting_duration);
//     // set_indexer_piston(false);
    
//     //Use this if your indexer is connected to your intake motor
//     // intake_timed(12000, shooting_duration);


//     pros::delay(time_between_shots);
//   }
// }


// void control_indexer() {
//   //Use this section if your indexer uses a dedicated motor
//   if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
//     set_indexer(12000);
//   } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
//     set_indexer(-12000);
//   } else {
//     set_indexer(0);
//   }
  
//   //Use this section if your indexer uses a piston
//   // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
//   //   //If you want to click once to extend the indexer and click again to retract it
//   //   toggle_indexer();
    
//   //   //If you want to only click once to both extend and then retract the indexer
//   //   // shoot(1, 250, 0);
//   // }
// }