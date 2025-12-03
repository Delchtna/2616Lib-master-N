#pragma once

#include "pros/motors.hpp"
#include "pros/misc.hpp"

class Catapult {
	public:
		Catapult(std::vector<int> motor_ports, char limit_switch_port,
				    pros::controller_digital_e_t automatic_button, pros::controller_digital_e_t manual_power_button, pros::controller_digital_e_t manual_toggle_button,
            pros::motor_gearset_e_t cartridge = pros::E_MOTOR_GEAR_100);

		void shoot_catapult();
		void full_power_motors();
		void stop_motors();

		void set_minimum_movement_time(long millis);
		bool is_limit_switch_pressed();


	private:
		std::vector<pros::Motor> motors;
		pros::controller_digital_e_t automatic_button;
		pros::controller_digital_e_t manual_power_button;
		pros::controller_digital_e_t manual_toggle_button;

		long minimum_movement_time = 250;
		bool manual_control_only = false;

		void catapult_task();
		void check_manual_control();
		void check_automatic_control();
};