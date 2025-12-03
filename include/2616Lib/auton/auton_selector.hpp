#pragma once

#include "api.h"


namespace Auton_Selector {

  //Auton subclass used to store information about a specific auton
  class Auton {
    public:
      Auton(std::string name, std::string description, std::function<void()> auton_call);
      std::string name;
      std::string description;
      std::function<void()> run;
  };


  void init(std::vector<Auton> autons);

  void run_selected_auton();
  void print_selected_auton();

  bool is_empty();
  int get_current_auton_page();
  std::string get_current_auton_name();
  std::string get_current_auton_description();

  void page_up();
  void page_down();

  //On-screen page changer buttons
  void draw_onscreen_buttons();
  void handle_onscreen_buttons();

  void limit_switch_initialize(pros::ADIDigitalIn* up_limit, pros::ADIDigitalIn* down_limit = nullptr);
  void limit_switch_task_func();

  void controller_print_auton_task_func();

  void enable_auton_selector();
  void disable_auton_selector();
  bool is_enabled();
}