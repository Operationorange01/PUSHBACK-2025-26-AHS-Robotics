#include "main.h"
#include <cmath>
#include "lemlib/api.hpp" // IWYU pragma: keep

// ==============================
// CONTROLLER
// ==============================
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ==============================
// MOTOR GROUPS
// ==============================
pros::MotorGroup leftMotors({-1, -11, -2}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({12, 3, 13}, pros::MotorGearset::blue);

// intake/outtake motors
pros::Motor n1(4, pros::MotorGearset::green);
pros::Motor n2(5, pros::MotorGearset::green);
pros::Motor n3(-9, pros::MotorGearset::green);
pros::Motor n4(-7, pros::MotorGearset::green);

// pistons
pros::ADIDigitalOut outtakePiston('B');
pros::ADIDigitalOut wingPiston('A');
pros::ADIDigitalOut tonguePiston('C');

// sensors
pros::Imu imu(19);
pros::Rotation horizontalEnc(10);
pros::Rotation verticalEnc(20);

// tracking wheels
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// ==============================
// DRIVETRAIN
// ==============================
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    13,
    lemlib::Omniwheel::NEW_325,
    450,
    4
);

// controllers
lemlib::ControllerSettings linearController(
    10, 0, 3, 0,
    0, 0,
    0, 0,
    0
);

lemlib::ControllerSettings angularController(
    2, 0, 10, 0,
    0, 0,
    0, 0,
    0
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
    linearController,
    angularController,
    sensors,
    &throttleCurve,
    &steerCurve
);

// ==============================
// AUTON PATHS
// ==============================
ASSET(path11_txt);
ASSET(path12_txt);
ASSET(path13_txt);
ASSET(path14_txt);
ASSET(path15_txt);
ASSET(path16_txt);
ASSET(path21_txt);
ASSET(path22_txt);
ASSET(path23_txt);
ASSET(path24_txt);
ASSET(path25_txt);
ASSET(path26_txt);

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
void autonLeft() {
    chassis.waitUntilDone();
    int autonSpeedL = 100;

    chassis.follow(path11_txt, 15, 4000, true);
    autonTongueAndFullMotor(true, -autonSpeedL, -autonSpeedL, -autonSpeedL, 0);

    chassis.follow(path12_txt, 15, 4000, true);
    autonTongueAndFullMotor(false, 0, 0, 0, 0);

    chassis.follow(path13_txt, 15, 4000, true);
    autonTongueAndFullMotor(true, -autonSpeedL, -autonSpeedL, -autonSpeedL, 0);

    chassis.follow(path14_txt, 15, 4000, true);
    autonTongueAndFullMotor(false, 0, 0, 0, 0);

    chassis.follow(path15_txt, 15, 4000, false);
    chassis.follow(path16_txt, 15, 4000, false);

    autonTongueAndFullMotor(false, autonSpeedL, autonSpeedL, autonSpeedL, autonSpeedL);
}

void autonRight() {
    chassis.waitUntilDone();
    int autonSpeedR = 100;

    chassis.follow(path21_txt, 15, 4000, true);
    autonTongueAndFullMotor(true, -autonSpeedR, -autonSpeedR, -autonSpeedR, 0);

    chassis.follow(path22_txt, 15, 4000, true);
    autonTongueAndFullMotor(false, 0, 0, 0, 0);

    chassis.follow(path23_txt, 15, 4000, true);
    autonTongueAndFullMotor(true, -autonSpeedR, -autonSpeedR, -autonSpeedR, 0);

    chassis.follow(path24_txt, 15, 4000, true);
    autonTongueAndFullMotor(false, 0, 0, 0, 0);

    chassis.follow(path25_txt, 15, 4000, false);
    chassis.follow(path26_txt, 15, 4000, false);

    autonTongueAndFullMotor(false, autonSpeedR, autonSpeedR, autonSpeedR, autonSpeedR);
}

void autonMiddle() {
    chassis.moveToPose(12, 12, 45, 3000);
}

// ==============================
// INITIALIZE (FIXED)
// ==============================
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

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
void competition_initialize() {}

// ==============================
// AUTONOMOUS
// ==============================
void autonomous() {
    autonLeft(); // CHANGE THIS IF NEEDED
}

// ==============================
// DRIVER CONTROL
// ==============================
double expo(int input, double k) {
    double x = input / 127.0;
    return (k * x * x * x + (1.0 - k) * x) * 127.0;
}

void opcontrol() {
    constexpr int INTAKEJOY_DEADZONE = 15;
    constexpr double INTAKE_EXPO_K = 0.6;
    constexpr int MAX_INTAKE_RPM = 200;

    bool outtakeDown = false;
    bool tongueOut = false;

    bool r1Prev = false;
    bool r2Prev = false;
    bool l2Prev = false;

    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        chassis.arcade(leftY, leftX);

        bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool l2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool l1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

        if ((r1 && !r1Prev) || (r2 && !r2Prev)) {
            outtakeDown = !outtakeDown;
            outtakePiston.set_value(outtakeDown);
        }

        r1Prev = r1;
        r2Prev = r2;

        if (l2 && !l2Prev) {
            tongueOut = !tongueOut;
            tonguePiston.set_value(tongueOut);
        }
        l2Prev = l2;

        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        if (r1) {
            outtakePiston.set_value(false);
            n1.move_velocity(200);
            n2.move_velocity(200);
            n3.move_velocity(200);
            n4.move_velocity(200);
        } 
        else if (r2) {
            outtakePiston.set_value(true);
            n1.move_velocity(0);
            n2.move_velocity(0);
            n3.move_velocity(0);
            n4.move_velocity(0);
            if (abs(rightY) > INTAKEJOY_DEADZONE) {
                int speed = static_cast<int>(expo(rightY, INTAKE_EXPO_K));
                int rpm = (speed * MAX_INTAKE_RPM) / 127;
                outtakePiston.set_value(false);
                n1.move_velocity(rpm);
                n2.move_velocity(rpm);
                n3.move_velocity(rpm);
                n4.move_velocity(0);
                }
        } 
        else if (abs(rightY) > INTAKEJOY_DEADZONE) {
            int speed = static_cast<int>(expo(rightY, INTAKE_EXPO_K));
            int rpm = (speed * MAX_INTAKE_RPM) / 127;
            outtakePiston.set_value(false);
            n1.move_velocity(rpm);
            n2.move_velocity(rpm);
            n3.move_velocity(rpm);
            n4.move_velocity(0);
        } 
        else {
            outtakePiston.set_value(false);
            n1.move_velocity(0);
            n2.move_velocity(0);
            n3.move_velocity(0);
            n4.move_velocity(0);
        }

        wingPiston.set_value(!l1);

        pros::delay(10);
    }
}
