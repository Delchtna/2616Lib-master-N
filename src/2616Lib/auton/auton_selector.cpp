#include "main.h"


//Change which auton is currently selected using on-screen buttons and/or limit switches
namespace Auton_Selector {

  std::vector<Auton> autons = {};
  int current_auton_page = 0;

  bool enabled = true; //If false, the auton selector does NOT print on the brain screen, and the auton page CAN NOT change
  bool locked = false; //If true, the auton selector DOES print on the brain screen, but the auton page CAN NOT change

  //Auton class constructor
  Auton::Auton(std::string name, std::string description, std::function<void()> callback) {
    this->name = name;
    this->description = description;
    this->run = callback;
  }

  void init(std::vector<Auton> new_autons) {
    autons = new_autons;
    current_auton_page = 0;

    printf("Auton selector initialized with %i autons!\n", autons.size());

    //Set up callback for the on-screen auton selector buttons
    pros::screen::touch_callback(handle_onscreen_buttons, pros::E_TOUCH_PRESSED);
  }


  bool is_empty() { return autons.size() == 0; }

  //Get the currently selected page (starting at 1)
  int get_current_auton_page() { return current_auton_page + 1; }

  //Get the name of the selected auton
  std::string get_current_auton_name() { return autons[current_auton_page].name; }

  //Get the description of the selected auton
  std::string get_current_auton_description() { return autons[current_auton_page].description; }

  //Print the selected auton name and description - only called inside of Chassis::update_brain_display()
  void print_selected_auton() {
    if (!enabled) { return; }
    if (is_empty()) {
      pros::screen::set_pen(COLOR_RED);
      pros::screen::print(pros::E_TEXT_MEDIUM, 6, "No autons!");
      return;
    }

    pros::screen::set_pen(COLOR_WHITE);

    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Auton %i:  %s", get_current_auton_page(), get_current_auton_name());

    std::string description = get_current_auton_description();
    auto newline_pos = description.find('\n');
    if (newline_pos != std::string::npos) {
      //Print description on 2 lines if description contains "\n"
      pros::screen::print(pros::E_TEXT_MEDIUM, 7, "  %s", description.substr(0, newline_pos));
      pros::screen::print(pros::E_TEXT_MEDIUM, 8, "  %s", description.substr(newline_pos + 1));
    } else {
      //Description does not contain "\n", so print everything on one line
      pros::screen::print(pros::E_TEXT_MEDIUM, 7, "  %s", description);
    }
  }

  //Run the selected auton
  void run_selected_auton() {
    if (is_empty() || locked) { return; }
    
    locked = true; //Prevent the auton page from being changed
    printf("Running auton %i: %s\n", get_current_auton_page(), get_current_auton_name().c_str());
    autons[current_auton_page].run();
  }


  //Increment the page (with wrapping) by pressing the on-screen button or using a limit switch
  void page_up() {
    if (is_empty() || locked) { return; }

    if (current_auton_page == autons.size() - 1) {
      current_auton_page = 0;
    } else {
      current_auton_page++;
    }

    printf("Selected auton %i:  %s\n", current_auton_page + 1, autons[current_auton_page].name.c_str());
  }

  //Decrement the page (with wrapping) by pressing the on-screen button or using a limit switch
  void page_down() {
    if (is_empty() || locked) { return; }
    
    if (current_auton_page == 0) {
      current_auton_page = autons.size() - 1;
    } else {
      current_auton_page--;
    }

    printf("Selected auton %i:  %s\n", current_auton_page + 1, autons[current_auton_page].name.c_str());
  }


  const int LEFT_BUTTON_X1 = 20;
  const int LEFT_BUTTON_X2 = 100;
  const int RIGHT_BUTTON_X1 = 120;
  const int RIGHT_BUTTON_X2 = 200;
  const int BUTTON_Y1 = 180;
  const int BUTTON_Y2 = 220;
  //Draw the two on-screen buttons that are used to change the selected auton
  void draw_onscreen_buttons() {
    pros::screen::set_pen(Chassis::get_auton_selector_button_color());
    pros::screen::fill_rect(LEFT_BUTTON_X1, BUTTON_Y1, LEFT_BUTTON_X2, BUTTON_Y2); //Left (page down) button 
    pros::screen::fill_rect(RIGHT_BUTTON_X1, BUTTON_Y1, RIGHT_BUTTON_X2, BUTTON_Y2); //Right (page up) button
  }

  //Check if either of the on-screen buttons are being pressed and change the selected auton accordingly
  void handle_onscreen_buttons() {
    if (!enabled) { return; }

    //Get coordinates of the touch
    pros::screen_touch_status_s_t status = pros::screen::touch_status();
    int touch_x = status.x;
    int touch_y = status.y;

    if (touch_x > LEFT_BUTTON_X1 && touch_x < LEFT_BUTTON_X2 && touch_y > BUTTON_Y1 && touch_y < BUTTON_Y2) {
      //Left button pressed
      page_down();
    } else if (touch_x > RIGHT_BUTTON_X1 && touch_x < RIGHT_BUTTON_X2 && touch_y > BUTTON_Y1 && touch_y < BUTTON_Y2) {
      //Right button pressed
      page_up();
    }
  }


  bool limit_switch_initialized = false;
  pros::ADIDigitalIn* page_up_limit_switch = nullptr;
  pros::ADIDigitalIn* page_down_limit_switch = nullptr;

  //Check for new presses on both of the limit switches if they exist, then change the selected auton page accordingly
  pros::Task limit_switch_task(limit_switch_task_func);
  void limit_switch_task_func() {
    while (true) {
      if (limit_switch_initialized && enabled) {

        if (page_up_limit_switch && page_up_limit_switch->get_new_press()) {
          page_up();
        } else if (page_down_limit_switch && page_down_limit_switch->get_new_press()) {
          page_down();
        }

      }/*  else {
        printf("Limit switch task func not enabled:   initialized=%i  enabled=%i\n", limit_switch_initialized, enabled);
      } */

      pros::delay(Util::DELAY_TIME);
    }
  }

  //Use one or two limit switches to control the auton selector
  void limit_switch_initialize(pros::ADIDigitalIn* page_up, pros::ADIDigitalIn* page_down) {
    printf("Initializing auton selector limit switch(es)...\n");

    if (!page_up && !page_down) {
      limit_switch_task.suspend();
      printf("No valid limit switch ports! Auton selector limit switch task suspended!");
      return;
    }

    if (page_up == page_down) {
      //If both switches refer to the same object, disable the down limit switch
      page_down = nullptr;
    }

    page_up_limit_switch = page_up;
    page_down_limit_switch = page_down;

    limit_switch_initialized = true;
    printf("Auton selector limit switch(es) initialized!\n");
  }


  //Print the currently selected auton onto the bottom line of the controller screen
  pros::Task controller_print_auton_task(controller_print_auton_task_func);
  void controller_print_auton_task_func() {
    while (true) {
      if (is_enabled()) {
        if (!is_empty()) {
          controller.print(2, 0, "%i:   %s", get_current_auton_page(), get_current_auton_name().c_str());
        } else {
          controller.print(2, 0, "No autons loaded!");
        }
      }

      //Auton has already run, so exit the task
      if (locked) {
        break;
      }

      pros::delay(50);
    }
  }


  //Enable the entire auton selector
  void enable_auton_selector() {
    if (enabled) {
      printf("Auton selector is already enabled!\n");
      return;
    }
    
    limit_switch_task.resume();
    
    enabled = true;

    printf("Enabled the auton selector!\n");
  }

  //Disable the entire auton selector
  void disable_auton_selector() {
    if (!enabled) {
      printf("Auton selector is already disabled!\n");
      return;
    }

    limit_switch_task.suspend();

    enabled = false;

    printf("Disabled the auton selector!\n");
  }

  //Check if the auton selector is enabled or not
  bool is_enabled() {
    return enabled;
  }


} //namespace Auton_Selector
