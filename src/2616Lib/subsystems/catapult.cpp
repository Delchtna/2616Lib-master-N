#include "main.h"

//TODO: test this class to make sure it all works correctly

pros::ADIDigitalIn catapultLimitSwitch('\0');

Catapult::Catapult(std::vector<int> motor_ports, char limit_switch_port, pros::controller_digital_e_t automatic_button, pros::controller_digital_e_t manual_power_button, pros::controller_digital_e_t manual_toggle_button, pros::motor_gearset_e_t cartridge): 
	automatic_button(automatic_button), manual_power_button(manual_power_button) {

  //Fill the motor vector
	for (auto i : motor_ports) {
		pros::Motor temp(abs(i), cartridge, Util::is_reversed(i));
		motors.push_back(temp);
	}

	//Instantiate the limit switch
	catapultLimitSwitch = pros::ADIDigitalIn(limit_switch_port);

	//Start main catapult task
	pros::Task catapult_task([this] { this->catapult_task(); });
}


//Main catapult task, which checks the catapult's automatic and manual controls
void Catapult::catapult_task() {
	while (true) {
		if (!Auton_Selector::is_enabled()) {
			check_automatic_control();
			check_manual_control();
		}

		pros::delay(Util::DELAY_TIME);
	}
}


//Shoot the catapult once
void Catapult::shoot_catapult() {
	long start = pros::millis();

	//Move the catapult for at least `minimum_ms` milliseconds, and until the limit switch is pressed
	while (pros::millis() < (start + minimum_movement_time) || !is_limit_switch_pressed()) {
		if (manual_control_only) { break; }

		full_power_motors();
	}

  //Limit switch is pressed, so stop moving the catapult
	stop_motors();
}

//Shoot catapult and automatically return to starting position
void Catapult::check_automatic_control() {
	if (controller.get_digital_new_press(automatic_button) && !manual_control_only) {
	//Manual control is disabled and automatic button was clicked, so shoot
		shoot_catapult();
	}
}

//Set power to the catapult using a button
void Catapult::check_manual_control() {
	if (controller.get_digital_new_press(manual_toggle_button)) {
	//Toggle manual control on or off when button pressed
		manual_control_only = !manual_control_only;
	}

	if (controller.get_digital(manual_power_button) && manual_control_only) {
	//Manual control is enabled and manual power button is being pressed, so move the catapult at full power
	full_power_motors();
	} else if (manual_control_only) {
	//Manual control is enabled but manual power button is not pressed, so stop the catapult
	  stop_motors();
  }
}

//Set all catapult motors to full power
void Catapult::full_power_motors() {
	for (auto i : motors) {
		i.move_voltage(12000);
	}
}

//Stop all the catapult motors
void Catapult::stop_motors() {
	for (auto i : motors) {
		i.move_velocity(0);
	}
}

//Set the minimum time for the catapult to be moving
void Catapult::set_minimum_movement_time(long millis) {
	minimum_movement_time = millis;
}

//Get the current value of the limit switch
bool Catapult::is_limit_switch_pressed() {
	return catapultLimitSwitch.get_value();
}