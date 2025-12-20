#include "main.h"
#include <cmath>
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-1, -11, -2}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({12, 3, 13}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// intake/outtake motors
pros::Motor n1(4, pros::MotorGearset::green);
pros::Motor n2(5, pros::MotorGearset::green);
pros::Motor n3(-9, pros::MotorGearset::green);
pros::Motor n4(-7, pros::MotorGearset::green);

pros::ADIDigitalOut outtakePiston('B');
pros::ADIDigitalOut wingPiston('A'); // wing piston ADI port
pros::ADIDigitalOut tonguePiston('C'); // pick correct port

// Inertial Sensor on port 10
pros::Imu imu(19);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(10);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(20);
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
lemlib::ExpoDriveCurve steerCurve(3, //the deadband now is 6. any movement lower than 6 will be ignored.
10, // minimum output where drivetrain will move out of 127
5 // expo curve gain
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
ASSET(path11_txt); // '.' replaced with "_" to make c++ happy
ASSET(path12_txt);
ASSET(path13_txt);
ASSET(path14_txt);
ASSET(path15_txt);
ASSET(path16_txt);
ASSET(path21_txt); // '.' replaced with "_" to make c++ happy
ASSET(path22_txt);
ASSET(path23_txt);
ASSET(path24_txt);
ASSET(path25_txt);
ASSET(path26_txt);

// function to make it easier
void autonTongueAndFullMotor(bool tong, int speed1, int speed2, int speed3, int speed4){
    tonguePiston.set_value(tong);
    n1.move_velocity(speed1);
    n2.move_velocity(speed2);
    n3.move_velocity(speed3);
    n4.move_velocity(speed4);
}

// define our three basic autonomous routines
void autonLeft() {
    // chassis.follow(path11_txt, 15, 3000, true);
    chassis.waitUntilDone(); // wait for path to finish

    //do intake/outtake action here
    chassis.follow(path11_txt, 15, 4000, true);
    autonTongueAndFullMotor(true, -100, -100, -100, 0);
    chassis.follow(path12_txt, 15, 4000, true);
    autonTongueAndFullMotor(false, 0, 0, 0, 0);
    chassis.follow(path13_txt, 15, 4000, true);
    autonTongueAndFullMotor(true, -100, -100, -100, 0);
    chassis.follow(path14_txt, 15, 4000, true);
    autonTongueAndFullMotor(false, 0, 0, 0, 0);
    chassis.follow(path15_txt, 15, 4000, false);
    autonTongueAndFullMotor(false, 0, 0, 0, 0);
    chassis.follow(path16_txt, 15, 4000, false);
    autonTongueAndFullMotor(false, 100, 100, 100, 100);

}

void autonRight() {
    // chassis.follow(path21_txt, 15, 3000, true);
    chassis.waitUntilDone(); // wait for path to finish

    //do intake/outtake action here
    chassis.follow(path21_txt, 15, 4000, true);
    autonTongueAndFullMotor(true, -100, -100, -100, 0);
    chassis.follow(path22_txt, 15, 4000, true);
    autonTongueAndFullMotor(false, 0, 0, 0, 0);
    chassis.follow(path23_txt, 15, 4000, true);
    autonTongueAndFullMotor(true, -100, -100, -100, 0);
    chassis.follow(path24_txt, 15, 4000, true);
    autonTongueAndFullMotor(false, 0, 0, 0, 0);
    chassis.follow(path25_txt, 15, 4000, false);
    autonTongueAndFullMotor(false, 0, 0, 0, 0);
    chassis.follow(path26_txt, 15, 4000, false);
    autonTongueAndFullMotor(false, 100, 100, 100, 100);

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

bool confirmed = false;
bool leftPrev = false, rightPrev = false, centerPrev = false;

while (!confirmed) {
uint32_t buttons = pros::lcd::read_buttons();
bool left = buttons & LCD_BTN_LEFT;
bool right = buttons & LCD_BTN_RIGHT;
bool center = buttons & LCD_BTN_CENTER;

// Cycle left
if (left && !leftPrev) {
selectedAuton--;
if (selectedAuton < 0) selectedAuton = NUM_AUTONS - 1;
pros::lcd::set_text(2, autonNames[selectedAuton]);
}

// Cycle right
if (right && !rightPrev) {
selectedAuton++;
if (selectedAuton >= NUM_AUTONS) selectedAuton = 0;
pros::lcd::set_text(2, autonNames[selectedAuton]);
}

// Confirm selection
if (center && !centerPrev) {
confirmed = true;
}

leftPrev = left;
rightPrev = right;
centerPrev = center;

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


/**
* Runs during auto
*
* This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
*/
void autonomous() {
    chassis.turnToHeading(90.0, 10000);

// ---- TEMP PID TEST ----
// move forward 24 inches   
/*
chassis.moveToPose(24, 0, 0, 3000);
chassis.waitUntilDone();
pros::delay(1000); // pause

// move back to start
chassis.moveToPose(0, 0, 0, 3000);
chassis.waitUntilDone();
pros::delay(1000);
// ---- END TEMP PID TEST ----
*/
}

// ==============================
// DRIVER CONTROL FUNCTIONS
// ==============================

double expo(int input, double k) {
double x = input / 127.0;
return (k * x * x * x + (1.0 - k) * x) * 127.0;
}
                                                                                                                                                                                                                                                                                         
void opcontrol() {
// ==============================
    // DRIVER CONTROL LOOP
    // CONSTANTS
// ==============================
constexpr int INTAKEJOY_DEADZONE = 15;
constexpr double INTAKE_EXPO_K = 0.6;
constexpr int MAX_INTAKE_RPM = 200;

    // ==============================
    // STATE TRACKING
    // ==============================
    bool outtakeDown = false;   // false = up, true = down
    bool tongueOut = false;

    bool r1Prev = false;
    bool r2Prev = false;
    bool l2Prev = false;

while (true) {
// ------------------------------
// DRIVE CONTROL
// ------------------------------
int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
chassis.arcade(leftY, leftX);

// ------------------------------
        // INTAKE/OUTTAKE CONTROL
        // READ BUTTONS
        // ------------------------------
        bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool l2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool l1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

        // ------------------------------
        // OUTTAKE PISTON TOGGLE (R1 / R2)
        // ------------------------------
        bool outtakeTogglePressed =
            (r1 && !r1Prev) ||
            (r2 && !r2Prev);

        if (outtakeTogglePressed) {
            outtakeDown = !outtakeDown;
            outtakePiston.set_value(outtakeDown);
        }

        r1Prev = r1;
        r2Prev = r2;

        // ------------------------------
        // TONGUE TOGGLE (L2)
        // ------------------------------
        if (l2 && !l2Prev) {
            tongueOut = !tongueOut;
            tonguePiston.set_value(tongueOut);
        }
        l2Prev = l2;

        // ------------------------------
        // INTAKE / OUTTAKE MOTORS
// ------------------------------
int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1); // Top outtake
        r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2); // Middle outtake

        if (r1) {
        // ---- TOP OUTTAKE ----
                    outtakePiston.set_value(false); // piston down
        n1.move_velocity(200);
        n2.move_velocity(200);
        n3.move_velocity(200);
                    n4.move_velocity(200); // n4 only runs for top outtake
                    n4.move_velocity(200);
        }
        else if (r2) {
        // ---- MIDDLE OUTTAKE ----
                    outtakePiston.set_value(true); // piston up
        n1.move_velocity(200);
        n2.move_velocity(200);
        n3.move_velocity(200);
                    n4.move_velocity(0); // n4 stays off
                    n4.move_velocity(0);
        }
        else if (abs(rightY) > INTAKEJOY_DEADZONE) {
        // ---- JOYSTICK INTAKE ----
        int speed = static_cast<int>(expo(rightY, INTAKE_EXPO_K));
        int speedRPM = (speed * MAX_INTAKE_RPM) / 127;
                    outtakePiston.set_value(false); // piston down

        n1.move_velocity(speedRPM);
        n2.move_velocity(speedRPM);
        n3.move_velocity(speedRPM);
                    n4.move_velocity(0); // n4 stays off
                    n4.move_velocity(0);
        }
        else {
                    // ---- STOP ALL INTAKE MOTORS ----
                    outtakePiston.set_value(false); // piston down
                    // ---- STOP INTAKE ----
        n1.move_velocity(0);
        n2.move_velocity(0);
        n3.move_velocity(0);
                    n4.move_velocity(0); // n4 stays off
                    n4.move_velocity(0);
        }

        // ------------------------------
                // WING CONTROL (Hold L1)
                // WING CONTROL (HOLD L1)
        // ------------------------------
                l1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
                wingPiston.set_value(!l1); // false when held (lower wing), true when released (raise wing)
                wingPiston.set_value(!l1);

                pros::delay(10); // small delay to prevent CPU overload
                pros::delay(10);
        }
}