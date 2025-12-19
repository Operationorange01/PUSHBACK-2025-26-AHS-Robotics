#include "main.h"
#include <cmath>
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-5, 4, -3}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({6, -9, 7}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// intake/outtake motors
pros::Motor n1(-1, pros::MotorGearset::green);
pros::Motor n2(2, pros::MotorGearset::green);
pros::Motor n3(9, pros::MotorGearset::green);
pros::Motor n4(-10, pros::MotorGearset::green);

pros::ADIDigitalOut outtakePiston('B');
pros::ADIDigitalOut wingPiston('A'); // wing piston ADI port

// Inertial Sensor on port 10
pros::Imu imu(10);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              13, // 13 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 360
                              4 // horizontal drift is 4.
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP) -- how hard it drives
                                            0, // integral gain (kI) -- ignore for now
                                            3, // derivative gain (kD) -- how fast it stops
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP) -- how hard it turns
                                             0, // integral gain (kI) -- ignore for now
                                             10, // derivative gain (kD) -- how fast it stops
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, //joystick deadband out of 127.
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(8, //the deadband now is 8. any movement lower than 8 will be ignored.
                                  10, // minimum output where drivetrain will move out of 127
                                  1.01 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

//------------------------
// OUR AUTONOMOUS MENU SETUP
//------------------------
int selectedAuton = 0; // we're gonna store which auton we pick here
std::string autonNames[] = {"Left", "Right", "Middle"}; // names for each auton, so we can display them
constexpr int NUM_AUTONS = 3; // total number of autonomous routines

// define our three basic autonomous routines
void autonLeft() {
    //Path 1.1
    chassis.follow(path11_txt, 15, 3000, true);
    chassis.waitUntilDone(); // wait for path to finish

    //do intake/outtake action here
    outtakePiston.set_value(true);
    pros::delay(1000); // wait for a second
    outtakePiston.set_value(false); // close piston again
}

void autonRight() {
    // right side routine, another sample move
    chassis.moveToPose(24, 24, 90, 3000); // move diagonally to right
}

void autonMiddle() {
    // middle routine, kinda whatever we want
    chassis.moveToPose(12, 12, 45, 3000);
}

// single-button menu for the brain
void selectAutonMenu() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Select Autonomous:");
    pros::lcd::set_text(4, "CENTER = confirm");
    pros::lcd::set_text(2, autonNames[selectedAuton]);

    bool longPress = false;
    while (!longPress) {
        uint32_t buttons = pros::lcd::read_buttons();
        if (buttons & LCD_BTN_CENTER) {
            pros::delay(500); // hold for half a second to confirm
            longPress = true;
        } else {
            selectedAuton++;
            if (selectedAuton >= NUM_AUTONS) selectedAuton = 0;
            pros::lcd::set_text(2, autonNames[selectedAuton]);
            pros::delay(300); // scroll delay
        }
        pros::delay(20);
    }
}

//------------------------
// END AUTON MENU SETUP
//------------------------

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });

    // run our auton menu here, before match starts
    selectAutonMenu(); // we pick our autonomous routine here
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get paths for pure pursuit
// this needs to be put outside a function
ASSET(path12_txt); // '.' replaced with "_" to make c++ happy
ASSET(path11_txt);
ASSET(path13_txt);

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {

    // ---- TEMP PID TEST ----
    // move forward 24 inches
    chassis.moveToPose(24, 0, 0, 3000);
    chassis.waitUntilDone();
    pros::delay(1000); // pause

    // move back to start
    chassis.moveToPose(0, 0, 0, 3000);
    chassis.waitUntilDone();
    pros::delay(1000);
    // ---- END TEMP PID TEST ----
}

// ==============================
// DRIVER CONTROL FUNCTIONS
// ==============================

double expo(int input, double k) {
    double x = input / 127.0;
    return (k * x * x * x + (1.0 - k) * x) * 127.0;
}

void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions for driving
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, leftX);

        // ==============================
        // INTAKE CONTROL (Joystick + Buttons)
        // ==============================
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        // Deadzone for intake joystick
        constexpr int INTAKEJOY_DEADZONE = 15;

        // Exponential factor for intake joystick
        constexpr double INTAKE_EXPO_K = 0.6;

        // DEADZONE
        if (abs(rightY) < INTAKEJOY_DEADZONE) {
            n1.move_velocity(0);
            n2.move_velocity(0);
            n3.move_velocity(0);
            n4.move_velocity(0);
            outtakePiston.set_value(false);
        } 
        else {
            int speed = static_cast<int>(expo(rightY, INTAKE_EXPO_K));

            // R2 → middle outtake
            if (r2) {
                outtakePiston.set_value(true);
                int upSpeed = abs(speed);

                n1.move_velocity(upSpeed);
                n2.move_velocity(upSpeed);
                n3.move_velocity(upSpeed);
                n4.move_velocity(0);
            }
            // R1 → top outtake
            else if (r1) {
                outtakePiston.set_value(false);
                int upSpeed = abs(speed);

                n1.move_velocity(upSpeed);
                n2.move_velocity(upSpeed);
                n3.move_velocity(upSpeed);
                n4.move_velocity(upSpeed);
            }
            // Normal intake/outtake
            else {
                outtakePiston.set_value(false);

                n1.move_velocity(speed);
                n2.move_velocity(speed);
                n3.move_velocity(speed);
                n4.move_velocity(0);
            }
        }

        // WING CONTROL (PNEUMATICS)
        // Hold L1 = wing deploys (down)
        // Release L1 = wing retracts (up)
        wingPiston.set_value(
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)
        );

        // small delay to prevent CPU overload
        pros::delay(10);
    }
}
