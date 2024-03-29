#include "main.h"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include <string>
#include <vector>
#include <math.h>
#include <string>
#include "okapi/api.hpp"
#include "okapi/api/chassis/controller/chassisControllerPid.hpp"
using namespace okapi;

pros::Motor Arm(19, false);
pros::Motor Intake(20, false);
pros::ADIDigitalOut Piston('B');

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 *
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	std::shared_ptr<ChassisController> bot = ChassisControllerBuilder()     
			.withMotors(7,-2,-12,11)  // front right and back right were reversed in order to go forward   
			// change P then D first then I only if necessary  
			//start with P I and D with zero 
			.withGains( //0.7, 0, 0.1 results: faster, shaking less violently 0//
				{1.0E-3, 0, 0}, // Distance controller gains 
				{0.0008, 0, 0}, // turn controller gains
				{0.00001, 0, 0.0000}	// Angle controller (helps bot drive straight)
				)
			.withMaxVelocity(150)
			// Green gearset, 4 inch wheel diam, 10 inch wheel track
			.withDimensions(AbstractMotor::gearset::green, {{4_in, 10_in}, imev5GreenTPR})
			.build();

pros::lcd::set_text(1, "THIS IS AUTON!");

bool pistonOpen = true; // closed wings
Piston.set_value(pistonOpen); // closed wings
pros::delay(500);
Arm.move_velocity(50); // keep arm up

bot->moveDistance(-12.5_in); // push red triball backwards
bot->moveDistance(4.8_in); // move forward to make space
bot->turnAngle(-102_deg); // turn 1
bot->moveDistance(35.5_in); // move forward
bot->turnAngle(-90.5_deg); // turn 2
bot->moveDistance(27.5_in); // move forward to triball
Arm.move_velocity(-50);     
pros::delay(500); // release arm

Intake.move_velocity(100); // intake triball
pros::delay(1000);

//Intake.move_velocity(100); // keep intake moving

bot->turnAngle(21_deg); // turn 3
bot->moveDistance(-4.6_ft); // move backward

bot->turnAngle(80_deg); // turn 4
bot->moveDistance(6_ft); // move forward
bot->turnAngle(-40_deg); // turn 5
bot->moveDistance(4_in); // adjust
bot->turnAngle(-65_deg); // turn 6 
bot->moveDistance(3.1_ft);
pistonOpen = !pistonOpen; 
Piston.set_value(pistonOpen); // open wings
bot->moveDistance(1.4_ft); // move to front of goal
bot->turnAngle(95_deg); // turn 7
// bot->moveDistance(-8_in);

Intake.move_velocity(-200); // push triball out
pros::delay(500);
bot->moveDistance(16_in); // drive forward into goal
// Intake.move_velocity(-200);
pros::delay(800);
Intake.move_velocity(0); // stop intake
pistonOpen = !pistonOpen; 
Piston.set_value(pistonOpen); // close wings
// bot->moveDistance(6_in);
bot->moveDistance(-10.5_in); // back up from goal
bot->turnAngle(-90_deg);
bot->moveDistance(5.1_ft);
bot->turnAngle(-88_deg);
bot->moveDistance(2.25_ft);
pros::delay(500);
Intake.move_velocity(100);
bot->moveDistance(10_in);

bot->moveDistance(-4_ft);
bot->turnAngle(-75_deg);
pistonOpen = !pistonOpen; 
Piston.set_value(pistonOpen); // open wings
bot->moveDistance(3_ft);
bot->turnAngle(100_deg);
bot->moveDistance(1_ft);

Intake.move_velocity(-200); // push triball out
pros::delay(500);
bot->moveDistance(16_in); // drive forward into goal
// Intake.move_velocity(-200);
pros::delay(800);
Intake.move_velocity(0);



}

void opcontrol() {

	pros::Motor FrontLeft(7, false);
	pros::Motor FrontRight(2, true);
	pros::Motor BackRight(12, false);
	pros::Motor BackLeft(11, true);
	pros::Motor Arm(19, false);
	pros::Motor Intake(20, false);
	pros::ADIDigitalOut Piston('B');


	int yMotion;
	int xMotion;
	int ArmVoltage = 30;
	bool pistonOpen = true;
	
	while (true)
	{
		pros::lcd::set_text(1, "READY TO DRIVE");
		pros::lcd::set_text(2, "Front Left Motor: " + std::to_string(FrontLeft.get_position()));
		pros::lcd::set_text(3, "Front Right Motor:" + std::to_string(FrontRight.get_position()));
		pros::lcd::set_text(4, "Back Left Motor:" + std::to_string(BackLeft.get_position()));
		pros::lcd::set_text(5, "Back Right Motor:" + std::to_string(BackRight.get_position()));
		pros::lcd::set_text(6, "Arm Motor:" + std::to_string(Arm.get_position()));
		pros::lcd::set_text(7, "Intake Motor:" + std::to_string(Intake.get_position()));
		

		pros::Controller master(pros::E_CONTROLLER_MASTER);
		// driving control code

		yMotion = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); // ik this looks wrong, but it works
		xMotion = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		

		int right = -xMotion + yMotion; //-power + turn
		int left = xMotion + yMotion;	// power + turn

		FrontLeft.move(left); // Swap negatives if you want the bot to drive in the other direction
		BackLeft.move(-left);
		BackRight.move(right);
		FrontRight.move(-right);


		// Intake Control
		if(master.get_digital(DIGITAL_R2))
			{
				Intake.move_velocity(160);
			}
		else if (master.get_digital(DIGITAL_R1))
			{
				Intake.move_velocity(-160);
			}
		else
		{
				Intake.move_velocity(0);
		}


		// Wings Piston Toggle
		if(master.get_digital_new_press(DIGITAL_L1))
		{ 
            pistonOpen = !pistonOpen; 
            Piston.set_value(pistonOpen); // Set piston state accordingly
			pros::delay(20);
		}


		// Drop Intake
		if(master.get_digital(DIGITAL_A))
		{
			Arm.move_velocity(-50); // Move arm down at start of program
			pros::delay(500);
		}
		else
		{
			Arm.move_velocity(0);
		}
	}

}