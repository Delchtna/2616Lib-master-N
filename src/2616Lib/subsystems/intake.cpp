#include "main.h"
#include "pros/misc.h"


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
void set_rollers(int voltage){
  if(voltage ==0 ){
    roller.move_velocity(0);
  }else {
    roller.move_voltage(voltage);
  }
}

void set_scoring(int voltage){
  set_rollers(voltage);
  set_intake(voltage);
}
void set_scoring(int v1, int v2){
  set_rollers(v1);
  set_intake(v2);
}
void control_intake() {
  if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)||controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      set_rollers(12000);
      set_intake(12000);
    }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      set_rollers(-12000);
      set_intake(-12000);
    }else{
      set_rollers(0);
      set_intake(0);
    }
 

}


