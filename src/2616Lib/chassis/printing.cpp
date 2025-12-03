#include "2616Lib/util/util.hpp"
#include "globals.hpp"
#include "main.h"
#include "pros/colors.h"


/**********************************************************************/
/*          This file handles printing everyhing on the brain         */
/*                screen while the program is running.                */
/**********************************************************************/


//todo: add documentation in main.cpp? directing users to this file to edit this stuff

// ************************ CONFIGURABLE VARIABLES ************************

//Motor temperatures and battery charge will be displayed on the screen when the auton selector is disabled.
//You should edit the letters that correspond to each motor (e.g. "RC" for right_center or "F" for flywheel), and the motor objects themselves, which are defined in globals.hpp and here.
//All chassis motors are accessed directly inside this function. These motors are set up on a 6 motor drive. If you have a 4 motor drive, replace the entries for RB (right_back) and LB (left_back).
//If any of the temperatures print as "2147483647", there is a problem with the relevant motor or how you're accessing it.
void Chassis::print_motor_temps() {
  pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "RF: %i   RC: %i   RB: %i",
                        (int) chassis.right_motors.at(0).get_temperature(), (int) chassis.right_motors.at(1).get_temperature(), (int) chassis.right_motors.at(2).get_temperature());

  pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 7, "LF: %i   LC: %i   LB: %i",
                        (int) chassis.left_motors.at(0).get_temperature(), (int) chassis.left_motors.at(1).get_temperature(), (int) chassis.left_motors.at(2).get_temperature());

  pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 8, "In: %i   B: %i%",
                       (int) intake.get_temperature(), (int) pros::battery::get_capacity());
}


//Change the color of the auton selector buttons
const uint32_t AUTON_SELECTOR_BUTTON_COLOR = COLOR_ORANGE;

//Change the color of the robot graphic on the screen
const uint32_t ROBOT_MAIN_COLOR = COLOR_LIME_GREEN;
const uint32_t ROBOT_FRONT_COLOR = COLOR_GREEN;
const uint32_t ROBOT_CENTER_COLOR = COLOR_GREEN_YELLOW;

//The robot is drawn as a square whose total length and width is robot_radius * 2
const int ROBOT_RADIUS = 10;

//Display the center or corner points of the robot icon on the field graphic
bool display_center_point = false;
bool display_corner_points = false;

//The number of times the screen refreshes per second. This is most stable when the value is low
const int BRAIN_FRAMES_PER_SECOND = 2;


// ************************************************************************


void Chassis::brain_printing_task_func() {
  while (true) {
    update_brain_display();

    pros::delay(1000 / BRAIN_FRAMES_PER_SECOND);
  }
}


double x, y, angle;

//Display odom variables and auton selector info on brain screen
void Chassis::update_brain_display() {
  long start = pros::millis();
  
  //Clear screen before printing again
  pros::screen::erase();

  if (imu_is_calibrating()) {
    //Odom isn't ready yet since IMUs are calibrating, so only show this message
    pros::screen::set_pen(COLOR_RED);
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 4, "IMUs are calibrating...");
    return;
  }

  //Draw auton selector or motor temps as defined by user
  if (Auton_Selector::is_enabled()) {
    Auton_Selector::print_selected_auton();
    Auton_Selector::draw_onscreen_buttons();
  } else {
    pros::screen::set_pen(COLOR_WHITE);
    print_motor_temps();
  }

  //Update odom variables
  if (update_odom.load()) {
    x = get_pose().x;
    y = get_pose().y;
    angle = get_pose().angle;
  } else {
    //Odom sensor disconnected, so don't update the stored coordinates
    pros::screen::set_pen(COLOR_RED);
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Odom sensor error!");
  }

  //Print odom variables
  print_pose_text();

  //Print odom values and odom graphic
  draw_field_graphic();
  draw_bot_on_field_graphic();

  // pros::screen::set_pen(COLOR_WHITE);
  // pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 8, "Time to execute: %i ms", pros::millis() - start);
}


//Print the robot's X, Y, and angle
void Chassis::print_pose_text() {
  pros::screen::set_pen(COLOR_WHITE);

  pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 0, "X: %.3f", x);
  pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 1, "Y: %.3f", y);
  pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 2, "Angle: %.3f", Util::to_deg(angle)); //Convert angle from radians to degrees
}


//The pixel at (0, 0) is the top left corner of the screen, and the total dimensions of the screen include this point
const int SCREEN_WIDTH = 480;
const int SCREEN_HEIGHT = 240;

const int SCREEN_RIGHT_EDGE = SCREEN_WIDTH - 1;
const int SCREEN_BOTTOM_EDGE = SCREEN_HEIGHT - 1;

// the size of graphic is 192 pixels wide by 192 pixels long, which leaves a 32 by 32 pixel square for each field tile (including borders)
const int FIELD_SIZE = 192;

const int FIELD_RIGHT_EDGE = SCREEN_RIGHT_EDGE - 25;
const int FIELD_LEFT_EDGE = FIELD_RIGHT_EDGE - FIELD_SIZE;
const int FIELD_TOP_EDGE = (SCREEN_HEIGHT - FIELD_SIZE) / 2;
const int FIELD_BOTTOM_EDGE = FIELD_TOP_EDGE + FIELD_SIZE;
const int FIELD_CENTER_X = (FIELD_RIGHT_EDGE + FIELD_LEFT_EDGE) / 2;
const int FIELD_CENTER_Y = (FIELD_TOP_EDGE + FIELD_BOTTOM_EDGE) / 2;

//Draw a vertically centered grid on the right side of the screen to represent the field, with thinner lines to represent the edges of tiles.
//Also draw important field elements on top of the field tiles
void Chassis::draw_field_graphic() {
  // ********** Field Tiles and Walls **********
  
  //Draw padding of 5 pixels around the edge of the field - this padding IS NOT included in the size of the field graphic
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::fill_rect(FIELD_LEFT_EDGE - 5, FIELD_TOP_EDGE - 5, FIELD_RIGHT_EDGE + 5, FIELD_BOTTOM_EDGE + 5);

  //Fill in background color of tiles
  pros::screen::set_pen(COLOR_GRAY);
  pros::screen::fill_rect(FIELD_LEFT_EDGE + 1, FIELD_TOP_EDGE + 1, FIELD_RIGHT_EDGE - 1, FIELD_BOTTOM_EDGE - 1);

  pros::screen::set_pen(COLOR_DARK_GRAY);

  //Draw 5 vertical lines as field tile borders
  for (int i = 1; i <= 5; i++) {
    pros::screen::draw_line(FIELD_LEFT_EDGE + (i * 32), FIELD_TOP_EDGE + 1, FIELD_LEFT_EDGE + (i * 32), FIELD_BOTTOM_EDGE - 1);
  }

  //Draw 5 horizontal lines as field tile borders
  for (int i = 1; i <= 5; i++) {
    pros::screen::draw_line(FIELD_LEFT_EDGE + 1, FIELD_TOP_EDGE + (i * 32), FIELD_RIGHT_EDGE - 1, FIELD_TOP_EDGE + (i * 32));
  }


  // ********** Field Elements **********
  //Red team stands on the bottom edge, blue team stands on the top edge

  //Autonomous field lines
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::draw_line(FIELD_LEFT_EDGE, FIELD_CENTER_Y+1, FIELD_RIGHT_EDGE, FIELD_CENTER_Y+1); //Bottom center line
  pros::screen::draw_line(FIELD_LEFT_EDGE, FIELD_CENTER_Y-1, FIELD_RIGHT_EDGE, FIELD_CENTER_Y-1); //Top center line
  pros::screen::draw_line(FIELD_LEFT_EDGE, FIELD_BOTTOM_EDGE-16, FIELD_RIGHT_EDGE, FIELD_BOTTOM_EDGE-16); 
  pros::screen::draw_line(FIELD_LEFT_EDGE, FIELD_TOP_EDGE+16, FIELD_RIGHT_EDGE, FIELD_TOP_EDGE+16); 
  pros::screen::draw_line(FIELD_LEFT_EDGE+16, FIELD_BOTTOM_EDGE, FIELD_LEFT_EDGE+16, FIELD_TOP_EDGE);
  pros::screen::draw_line(FIELD_RIGHT_EDGE-16, FIELD_BOTTOM_EDGE, FIELD_RIGHT_EDGE-16, FIELD_TOP_EDGE); 

  //Draw center hang bar with thick border.
  pros::screen::set_pen(0x00ECFA0D);
  for (int i = -2; i <= 2; i++) {
    pros::screen::draw_line(FIELD_CENTER_X-32, FIELD_CENTER_Y + i, FIELD_CENTER_X, FIELD_CENTER_Y+32+i); 
    pros::screen::draw_line(FIELD_CENTER_X, FIELD_CENTER_Y -32 + i, FIELD_CENTER_X+32, FIELD_CENTER_Y+i);
    pros::screen::draw_line(FIELD_CENTER_X, FIELD_CENTER_Y +32 + i, FIELD_CENTER_X+32, FIELD_CENTER_Y+i);
    pros::screen::draw_line(FIELD_CENTER_X-32, FIELD_CENTER_Y + i, FIELD_CENTER_X, FIELD_CENTER_Y -32 + i); 
  }

  //Draw labels indicating positive/negative zone
  pros::screen::set_pen(COLOR_BLACK);
  for (int i = 0; i < 3; i++) {
    pros::screen::draw_line(FIELD_LEFT_EDGE-i, FIELD_BOTTOM_EDGE-8, FIELD_LEFT_EDGE-i, FIELD_BOTTOM_EDGE); //Top left blue match load
    pros::screen::draw_line(FIELD_RIGHT_EDGE+i, FIELD_BOTTOM_EDGE-8, FIELD_RIGHT_EDGE+i, FIELD_BOTTOM_EDGE); //Top left blue match load
    pros::screen::draw_line(FIELD_LEFT_EDGE-i, FIELD_TOP_EDGE+8, FIELD_LEFT_EDGE-i, FIELD_TOP_EDGE); //Top left blue match load
    pros::screen::draw_line(FIELD_RIGHT_EDGE+i, FIELD_TOP_EDGE+8, FIELD_RIGHT_EDGE+i, FIELD_TOP_EDGE); //Top left blue match load


    pros::screen::draw_line(FIELD_LEFT_EDGE, FIELD_BOTTOM_EDGE+i, FIELD_LEFT_EDGE+8, FIELD_BOTTOM_EDGE+i);
    pros::screen::draw_line(FIELD_LEFT_EDGE, FIELD_TOP_EDGE-i, FIELD_LEFT_EDGE+8, FIELD_TOP_EDGE-i);

    pros::screen::draw_line(FIELD_RIGHT_EDGE, FIELD_BOTTOM_EDGE+i, FIELD_RIGHT_EDGE-8, FIELD_BOTTOM_EDGE+i);
    pros::screen::draw_line(FIELD_RIGHT_EDGE, FIELD_TOP_EDGE-i, FIELD_RIGHT_EDGE-8, FIELD_TOP_EDGE-i);
  }
}


//Draw the bot's position on top of the position graphic using the X and Y coords on a cartesian plane representing the field.
//Using adjusted coordinates, the point (0, 0) is the center of the field
void Chassis::draw_bot_on_field_graphic() {
  //Scale the bot's odom position into a LOCAL position adjusted to the scale of the graphic: https://stats.stackexchange.com/questions/281162/scale-a-number-between-a-range
  int odom_range = 72 + (ROBOT_RADIUS / 2); //The real life field is 144" x 144" (12' x 12'), so the X and Y values should always be in the range (-72, 72) inches, then adjust a bit for the robot width
  float adjusted_x = (x + odom_range) / (odom_range * 2) * FIELD_SIZE - (FIELD_SIZE / 2.0);
  float adjusted_y = (y + odom_range) / (odom_range * 2) * FIELD_SIZE - (FIELD_SIZE / 2.0);

  //Convert local position into absolute position
  float robot_center_x = FIELD_CENTER_X + adjusted_x;
  float robot_center_y = FIELD_CENTER_Y - adjusted_y;

  //Set the X and Y coords of the 4 corners of the bot BEFORE rotating in this 2D array, the top row (index 0) is X coords and the bottom row (index 1) is Y coords.
  //The points are in this order:  top left, top right, bottom right, bottom left
  float robot_corners[2][4] = {
    {0 - ROBOT_RADIUS, 0 + ROBOT_RADIUS, 0 + ROBOT_RADIUS, 0 - ROBOT_RADIUS},
    {0 - ROBOT_RADIUS, 0 - ROBOT_RADIUS, 0 + ROBOT_RADIUS, 0 + ROBOT_RADIUS}
  };
  float rotated_robot_corners[2][4];


  //Rotate the corner points around the center point, then move them to the correct location in the graphic: https://en.wikipedia.org/wiki/Rotation_matrix#Non-standard_orientation_of_the_coordinate_system
  //A positive angle should mean clockwise rotation, but the brain screen coordinate system is inverted (the positive Y axis is down), so use the standard version of the rotation matrix (where a positive angle means counterclockwise rotation)
  for (int i = 0; i < 4; i++) {        
    //X coordinate
    rotated_robot_corners[0][i] = Util::x_rotate_point(robot_corners[0][i], robot_corners[1][i], angle, true) + robot_center_x;
    //Y coordinate
    rotated_robot_corners[1][i] = Util::y_rotate_point(robot_corners[0][i], robot_corners[1][i], angle, true) + robot_center_y;
  }


  //Draw the angled robot square by drawing lines using many different points on the left and right edges of the robot square
  int num_points = 2000;
  //Subtract the top left and bottom left X and Y values, then divide by num_points to get the change in X and Y for each line
  float x_step = (rotated_robot_corners[0][3] - rotated_robot_corners[0][0]) / num_points;
  float y_step = (rotated_robot_corners[1][3] - rotated_robot_corners[1][0]) / num_points;
  for (int i = 0; i < num_points; i++) {
    //Get the left and right points by adding the X and Y step to the top left and top right robot square corners
    float left_point_x = rotated_robot_corners[0][0] + x_step * i;
    float left_point_y = rotated_robot_corners[1][0] + y_step * i;
    float right_point_x = rotated_robot_corners[0][1] + x_step * i;
    float right_point_y = rotated_robot_corners[1][1] + y_step * i;

    if (i < num_points / 4) {
      pros::screen::set_pen(ROBOT_FRONT_COLOR); //Make the top quarter of the robot a different color than the rest of it
    } else {
      pros::screen::set_pen(ROBOT_MAIN_COLOR);
    }

    pros::screen::draw_line(left_point_x, left_point_y, right_point_x, right_point_y);
  }
  

  //Draw the corners and center points of the robot
  pros::screen::set_pen(ROBOT_CENTER_COLOR);
  if (display_corner_points) {
    for (int i = 0; i < 4; i++) {
      pros::screen::draw_pixel(rotated_robot_corners[0][i], rotated_robot_corners[1][i]);
    }
  }
  if (display_center_point) {
    pros::screen::fill_circle(robot_center_x, robot_center_y, 4);
  }

  
  //Print the robot as a solid rectangle - DOES NOT DISPLAY ROTATION
  //pros::screen::fill_rect(robot_center_x - robot_radius, robot_center_y - robot_radius, robot_center_x + robot_radius, robot_center_y + robot_radius);
}


uint32_t Chassis::get_auton_selector_button_color() {
  return AUTON_SELECTOR_BUTTON_COLOR;
}