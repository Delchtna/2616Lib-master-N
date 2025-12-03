#include "main.h"
#include "pistons.hpp"
#include "pros/adi.hpp"

//Example impelmentation of a piston subsystem.
void control_wings(){
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !SHIFT_KEY){
    set_wings(true);

  } else {
    set_wings(false);
  }
}
bool back_wing_state = false;

void control_back_wings(){
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && SHIFT_KEY){
    back_wing_state = true;
    set_intake(-12000);
   
  }
  else{
     back_wing_state = false;
  }
   back_wing.set_value(back_wing_state);
}

void set_wings(bool value){
  right_wing.set_value(value);
  left_wing.set_value(value);
}

void toggle_back_wing(){
  back_wing_state = !back_wing_state;
  back_wing.set_value(back_wing_state);

}

bool endgame_state = true;
bool completed = false;
void control_endgame(double start){
    if (((pros::millis() - start) > (45 *1000)) && completed == false){
      endgame_state = true;
      completed = true;
      // std::cout << start << std::endl;
      // std::cout << pros::millis() << std::endl;
      std::cout << pros::millis() - start << " " << endgame_state << std::endl;
       endgame.set_value(endgame_state);
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) && SHIFT_KEY){
      endgame_state = !endgame_state;
      endgame.set_value(endgame_state);
  }
}

void control_endgames(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) && SHIFT_KEY){
      endgame_state = !endgame_state;
      endgame.set_value(endgame_state);
  }
}

bool low_hang_state = false;
void control_low_hang(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
      low_hang_state = !low_hang_state;
      low_hang.set_value(low_hang_state);
  }
  
}

bool redirect_state = false;
  void control_redirect(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
      redirect_state = !redirect_state;
      redirect.set_value(redirect_state);
    }
  }

  bool tounge_state = false;
  void control_tounge(){
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
      tounge_state = !tounge_state;
      tounge.set_value(tounge_state);
    }
  }

  bool intake_state = false;
  void control_raise_intake(){
   if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
    intake_state = !intake_state;
    raise_intake.set_value(intake_state);
   }
  }
  
