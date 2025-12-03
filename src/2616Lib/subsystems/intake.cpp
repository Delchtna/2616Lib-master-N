#include "main.h"


//Spin the intake at a certain voltage. Voltage range is [-12000, 12000], and 0 stops the motor.
void set_intake(int voltage) {
  if (voltage == 0) {
    intake.move_velocity(0);
  } else {
    intake.move_voltage(voltage);
  }
}

//Spin the intake at a certain voltage for `millis` milliseconds, then stop
void intake_timed(int voltage, long millis) {
  set_intake(voltage);
  pros::delay(millis);
  set_intake(0);
}

//Check for forward or reverse intake buttons being held, and stop moving otherwise
void control_intake() {
  
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !SHIFT_KEY) {
    set_intake(12000);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && !SHIFT_KEY) {
    set_intake(-12000);
  }else if ( (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && SHIFT_KEY)){
    set_intake(-12000);
  }
  
  else {

    set_intake(0);
  }
  

}

void set_top_rollers(int voltage){
    if(voltage ==0){
      top_roller.move_velocity(0);
    }else{
      top_roller.move_voltage(voltage);
    }
  }
  
  void set_bottom_rollers(int voltage){
    if(voltage == 0){
      bottom_roller.move_velocity(0);
    }else{
      bottom_roller.move_voltage(voltage);
    }
  }
  void set_back_rollers(int voltage){
    if(voltage == 0){
      back_roller.move_velocity(0);
    }else{
      back_roller.move_voltage(voltage);
    }
  }

  void set_rollers(int voltage){
    set_back_rollers(voltage);
    set_top_rollers(voltage * .75);
    set_bottom_rollers(voltage);
  }

  void score_mid(int voltage){
    set_back_rollers(voltage);
    set_top_rollers(-voltage * .5);
    set_bottom_rollers(voltage * .75);
  }

  void control_rollers(){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
      set_rollers(12000);
    }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      score_mid(12000);
    }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      set_rollers(-12000);
    }else{
      set_rollers(0);
    }
  }