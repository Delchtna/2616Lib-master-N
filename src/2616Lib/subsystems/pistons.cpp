#include "globals.hpp"
#include "main.h"
#include "pistons.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"

//Example impelmentation of a piston subsystem.




  void control_tounge(){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
      tounge.set_value(true);
    }else{
      tounge.set_value(false);
    }
  }

  bool lift_state = true;
  void control_lift(){
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
      lift_state = !lift_state;
      roller_lift.set_value(lift_state);
      descore_lift.set_value(lift_state);
    }
  }


  void control_descore(){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
      descore.set_value(true);
    }else{
      descore.set_value(false);
    }
  }

  void control_holder(){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
      holder.set_value(false);
    }else{
      holder.set_value(true);
    }
  }

  