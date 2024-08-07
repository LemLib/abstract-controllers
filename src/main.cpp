#include "main.h"
#include "controller/feedforward.hpp"
#include "controller/fusions.hpp"
#include "controller/pid.hpp"
#include "controller/slew.hpp"
#include "units/units.hpp"
#include <functional>

Force test(Length, controllers::Controller<Length, Length, Voltage>& a,
           controllers::Controller<Length, Length, LinearVelocity>& b) {
    return 0_N;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    controllers::pidConfig<Length, Voltage> config {1, 2, 3};
    controllers::PID pid(config, 10_in);
    controllers::Slewed testSlewed(pid, 0.1_volt);
    controllers::AdditiveFusionController addTest(pid, testSlewed);
    controllers::LinearFeedForward<Voltage, LinearVelocity> ff(1_volt, 3 * m / sec / volt);
    controllers::ChainedFusionController chainedTest(addTest, ff);
    std::function<Force(Length, controllers::Controller<Length, Length, Voltage>&,
                        controllers::Controller<Length, Length, LinearVelocity>&)>
        b = test;
    controllers::FunctionalChainedController functionalTest(addTest, chainedTest, b, 5_m);
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
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::MotorGroup left_mg({1, -2, 3}); // Creates a motor group with forwards ports 1 & 3 and reversed port 2
    pros::MotorGroup right_mg({-4, 5, -6}); // Creates a motor group with forwards port 5 and reversed ports 4 & 6

    while (true) {
        pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                         (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                         (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0); // Prints status of the emulated screen LCDs

        // Arcade control scheme
        int dir = master.get_analog(ANALOG_LEFT_Y); // Gets amount forward/backward from left joystick
        int turn = master.get_analog(ANALOG_RIGHT_X); // Gets the turn left/right from right joystick
        left_mg.move(dir - turn); // Sets left motor voltage
        right_mg.move(dir + turn); // Sets right motor voltage
        pros::delay(20); // Run for 20 ms then update
    }
}