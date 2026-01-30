#include "main.h"
#include <cmath>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/misc.h"

// ==============================
// CONTROLLER
// ==============================
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ==============================
// MOTOR GROUPS
// ==============================
pros::MotorGroup leftMotors({-18, -19, -20}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({11, 12, 13}, pros::MotorGearset::blue);

// intake/outtake motors
pros::Motor n1(-1, pros::MotorGearset::green);
pros::Motor n2(-2, pros::MotorGearset::green);
pros::Motor n3(-3, pros::MotorGearset::green);
pros::Motor n4(-4, pros::MotorGearset::green);

// pistons
pros::ADIDigitalOut outtakePiston('B');
pros::ADIDigitalOut wingPiston('A');
pros::ADIDigitalOut tonguePiston('D');
pros::ADIDigitalOut doorPiston('C');

// sensors
pros::Imu imu(19);
pros::Rotation horizontalEnc(-10);
pros::Rotation verticalEnc(-20);

// tracking wheels
// tracking wheels (Changed from NEW_275 to 2.0)
lemlib::TrackingWheel horizontal(&horizontalEnc, 2.0, -5.75);
lemlib::TrackingWheel vertical(&verticalEnc, 2.0, -2.5);

// ==============================
// DRIVETRAIN
// ==============================
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    13,
    2.0, // Change this from NEW_325 to 2.0 if your drive wheels are 2 inches
    450,
    2 // This is your "chase power" - usually left at 2
);

// controllers
lemlib::ControllerSettings linear_controller(
    10,
    0,
    7,
    0, //average steady state error * 1.5
    1, //restore to 1 after PID tuning
    100, //restore to 100 after PID tuning
    3, //restore to 3 after PID tuning
    500, //restore to 500 after PID tuning
    0 //ignore for now
);

lemlib::ControllerSettings angular_controller(
    4,
    0,
    15,
    0, //average steady state error * 1.5
    1, //restore to 1 after PID tuning
    100, //restore to 100 after PID tuning
    3, //restore to 3 after PID tuning
    500, //restore to 500 after PID tuning
    0 //ignore for now
);

// odom sensors
lemlib::OdomSensors sensors(
    &vertical,
    nullptr,
    &horizontal,
    nullptr,
    &imu
);

// drive curves
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 5, 1);

// chassis
lemlib::Chassis chassis(
    drivetrain,
    linear_controller,
    angular_controller,
    sensors,
    &throttleCurve,
    &steerCurve
);

// ==============================
// AUTON PATHS
// ==============================





// ==============================
// AUTON HELPERS
// ==============================
void autonTongueAndFullMotor(bool tong, int s1, int s2, int s3, int s4) {

    tonguePiston.set_value(tong);
    n1.move_velocity(s1);
    n2.move_velocity(s2);
    n3.move_velocity(s3);
    n4.move_velocity(s4);
}

// ==============================
// AUTON ROUTINES
// ==============================
/*
void autonLeft() {
    int autonMotorSpeedL = 100;

    // Go to center blocks and start intake
    chassis.follow(path11_txt, 15, 4000, true);
    chassis.waitUntilDone();
    //autonTongueAndFullMotor(true, -autonMotorSpeedL, -autonMotorSpeedL, -autonMotorSpeedL, 0);

    // Move over the middle blocks
    chassis.follow(path12_txt, 15, 4000, true);
    chassis.waitUntilDone();
    //pros.wait(2000)
    //autonTongueAndFullMotor(false, 0, 0, 0, 0);

    // Line up to the match loader
    chassis.follow(path13_txt, 15, 4000, true);
    chassis.waitUntilDone();

    // Move to the match loader and intake blocks
    chassis.follow(path14_txt, 15, 4000, true);
    chassis.waitUntilDone();
    //autonTongueAndFullMotor(true, -autonMotorSpeedL, -autonMotorSpeedL, -autonMotorSpeedL, 0);
    //pros.wait(5000)

    // Move backward
    chassis.follow(path15_txt, 15, 4000, false);
    chassis.waitUntilDone();
    //autonTongueAndFullMotor(false, 0, 0, 0, 0);

    // Move to the side goals and dispense
    chassis.follow(path16_txt, 15, 4000, false);
    chassis.waitUntilDone();
    //autonTongueAndFullMotor(false, autonMotorSpeedL, autonMotorSpeedL, autonMotorSpeedL, autonMotorSpeedL);
}
*/
/*
void autonLeft() {
    int autonMotorSpeedL = 100;
    float maxLeftSpeed = 80;  
    autonTongueAndFullMotor(true, 0, 0, 0, 0);

    // Go to center blocks and start intake
    chassis.moveToPose(-35, 22.5, 80, 3000, {.forwards = true, .maxSpeed = maxLeftSpeed});
    chassis.waitUntilDone();
    autonTongueAndFullMotor(true,-autonMotorSpeedL, -autonMotorSpeedL, -autonMotorSpeedL, -autonMotorSpeedL);

    // Move over the middle blocks
    chassis.moveToPose(-25, 22.5, 90, 3000, {.forwards = true, .maxSpeed = maxLeftSpeed});
    chassis.waitUntilDone();
    pros::delay(1000);
    autonTongueAndFullMotor(true, 0, 0, 0, 0);
    */
/*
    // Line up to the match loader
    chassis.moveToPose(-46, 47, 285, 5000, {.forwards = true, .maxSpeed = maxLeftSpeed});
    chassis.waitUntilDone();

    // Move to the match loader and intake blocks
    chassis.moveToPose(-56, 47, 270, 5000, {.forwards = true, .maxSpeed = maxLeftSpeed});
    chassis.waitUntilDone();
    //autonTongueAndFullMotor(true, -autonMotorSpeedL, -autonMotorSpeedL, -autonMotorSpeedL, -autonMotorSpeedL);
    //pros.wait(2000)

    // Move backward
    chassis.moveToPose(-40, 47, 270, 5000, {.forwards = false, .maxSpeed = maxLeftSpeed});
    chassis.waitUntilDone();
    //autonTongueAndFullMotor(false, 0, 0, 0, 0);

    // Move to the side goals and dispense
    chassis.moveToPose(-33, 47, 270, 5000, {.forwards = false, .maxSpeed = maxLeftSpeed});
    chassis.waitUntilDone();
    //autonTongueAndFullMotor(false, autonMotorSpeedL, autonMotorSpeedL, autonMotorSpeedL, autonMotorSpeedL);*/


/*
void autonRight() {
    int autonMotorSpeedR = 100;

    chassis.follow(path21_txt, 12, 4000, true);
    autonTongueAndFullMotor(true, -autonMotorSpeedR, -autonMotorSpeedR, -autonMotorSpeedR, 0);
    chassis.waitUntilDone();

    chassis.follow(path22_txt, 12, 4000, true);
    autonTongueAndFullMotor(false, 0, 0, 0, 0);
    chassis.waitUntilDone();

    chassis.follow(path23_txt, 12, 4000, true);
    autonTongueAndFullMotor(true, -autonMotorSpeedR, -autonMotorSpeedR, -autonMotorSpeedR, 0);
    chassis.waitUntilDone();

    chassis.follow(path24_txt, 12, 4000, true);
    autonTongueAndFullMotor(false, 0, 0, 0, 0);
    chassis.waitUntilDone();
   
    chassis.follow(path25_txt, 12, 4000, false);
    chassis.waitUntilDone();

    chassis.follow(path26_txt, 12, 4000, false);
    chassis.waitUntilDone();

    autonTongueAndFullMotor(false, autonMotorSpeedR, autonMotorSpeedR, autonMotorSpeedR, autonMotorSpeedR);
}


void autonMiddle() {
    chassis.moveToPose(12, 12, 45, 3000);
}
*/
// ==============================
// INITIALIZE (FIXED)
// ==============================
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    chassis.setPose( -63, 18.5, 90);//temporary for PID tuning
    tonguePiston.set_value(true); // Tongue Up
    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::delay(50);
        }
    });
   
}

void disabled() {}
void competition_initialize() {
    tonguePiston.set_value(true); // Tongue Up
}

// move forward while intaking blocks
void autonDriveAndIntake(double targetX, double targetY, int timeoutMs, int intakeSpeed = 200) {
    // start intake motors
    n1.move_velocity(intakeSpeed);
    n2.move_velocity(intakeSpeed);
    n3.move_velocity(intakeSpeed);
    n4.move_velocity(0); // outtake stays off

    // start driving to the target
    chassis.moveToPose(targetX, targetY, chassis.getPose().theta, timeoutMs);

    // wait until drive is done
    chassis.waitUntilDone();

    // stop intake motors
    n1.move_velocity(0);
    n2.move_velocity(0);
    n3.move_velocity(0);
}

// turn then drive
void autonTurnAndDrive(double headingDeg, double targetX, double targetY, int timeoutMs) {
    chassis.turnTo(headingDeg);       // rotate in place
    chassis.waitUntilDone();

    chassis.moveToPose(targetX, targetY, headingDeg, timeoutMs);  // drive forward
    chassis.waitUntilDone();
}

// outtake all blocks
void autonOuttake(int speed = 200) {
    n1.move_velocity(speed);
    n2.move_velocity(speed);
    n3.move_velocity(speed);
    n4.move_velocity(speed);
    pros::delay(2000); // run for 2 seconds, adjust as needed
    n1.move_velocity(0);
    n2.move_velocity(0);
    n3.move_velocity(0);
    n4.move_velocity(0);
}


// ==============================
// AUTONOMOUS
// ==============================
void autonomous() {
    // set start pose
    chassis.setPose(-153, -43, 0);

    // drive to blocks while intaking
    autonDriveAndIntake(-76, -59, 3000);

    // after intake leg
    autonTurnAndDrive(???, -76.743, -121.431, 4000); // replace ??? with the turn angle

    // turn to long goal and move to deposit
    autonTurnAndDrive(-90, -61.261, -121.431, 3000); // turn to face goal and drive
    autonOuttake(200); // deposit all blocks

}


// ==============================
// DRIVER CONTROL
// ==============================
double expo(int input, double k) {
    double x = input / 127.0;
    return (k * x * x * x + (1.0 - k) * x) * 127.0;
}

void opcontrol() {
    // Constants
    constexpr int INTAKEJOY_DEADZONE = 15;
    constexpr double INTAKE_EXPO_K = 0.6;
    constexpr int MAX_INTAKE_RPM = 200;
    constexpr int MANUAL_OUTTAKE_SPEED = 200;

    // Toggle states
    bool outtakeDown = false;
    bool tongueState = false; // false = Up (default), true = Down

    // Previous button states
    bool r2Prev = false;
    bool l2Prev = false;

    while (true) {
        // ===== 1. DRIVE CONTROL =====
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
       
        // Using expo on leftX to make turning less sensitive
        chassis.arcade(leftY, expo(leftX, 0.45));

        // ===== 2. INPUT DATA =====
        bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool l1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool l2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // This variable now only depends on the R1 button
        bool isOuttaking = r1;

        // ===== 3. INTAKE / OUTTAKE MOTOR LOGIC =====
        if (r1) {
            // Manual Outtake via Button
            n1.move_velocity(MANUAL_OUTTAKE_SPEED);
            n2.move_velocity(MANUAL_OUTTAKE_SPEED);
            n3.move_velocity(MANUAL_OUTTAKE_SPEED);
            n4.move_velocity(-MANUAL_OUTTAKE_SPEED);
        }
        else if (r2) {
            // Manual Outtake via Button
            n1.move_velocity(MANUAL_OUTTAKE_SPEED);
            n2.move_velocity(MANUAL_OUTTAKE_SPEED);
            n3.move_velocity(MANUAL_OUTTAKE_SPEED);
            n4.move_velocity(MANUAL_OUTTAKE_SPEED);
        }
        else if (std::abs(rightY) > INTAKEJOY_DEADZONE) {
            // Intake/Outtake via Joystick (Does NOT open the door)
            int speed = static_cast<int>(expo(rightY, INTAKE_EXPO_K));
            int rpm = (speed * MAX_INTAKE_RPM) / 127;

            n1.move_velocity(rpm);
            n2.move_velocity(rpm);
            n3.move_velocity(rpm);
            n4.move_velocity(0);
        }
        else {
            n1.move_velocity(0);
            n2.move_velocity(0);
            n3.move_velocity(0);
            n4.move_velocity(0);
        }

        // ===== 4. PISTON CONTROL =====
       
        doorPiston.set_value(!isOuttaking);

        // TONGUE: Toggle logic
        if (l2 && !l2Prev) {
            tongueState = !tongueState;
        }
        l2Prev = l2;
        // Adjust this '!' if the tongue is starting in the wrong position too
        tonguePiston.set_value(!tongueState);

        // WINGS
        wingPiston.set_value(!l1);

        // OUTTAKE PISTON: Toggle logic
        if (r2 && !r2Prev) {
            outtakeDown = !outtakeDown;
            outtakePiston.set_value(outtakeDown);
        }
        r2Prev = r2;

        pros::delay(10);
    }
}
