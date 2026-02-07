#include "main.h"
#include <cmath>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
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
pros::Motor n3(3, pros::MotorGearset::green);

// pistons
pros::ADIDigitalOut outtakePiston('B');
pros::ADIDigitalOut wingPiston('A');
pros::ADIDigitalOut tonguePiston('D');
pros::ADIDigitalOut doorPiston('C');

// sensors
pros::Imu imu(9);
// horizontalEnc removed
pros::Rotation verticalEnc(15);

// tracking wheels
// horizontal tracking wheel removed
lemlib::TrackingWheel vertical(&verticalEnc, 2.0, 0);

// ==============================
// DRIVETRAIN
// ==============================
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    12,
    3.25, 
    450,
    2 
);

// controllers
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              36, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


lemlib::OdomSensors sensors(
    &vertical, // Slot 1: Vertical tracking wheel (YOUR WHEEL GOES HERE)
    nullptr,   // Slot 2: Second vertical wheel
    nullptr,   // Slot 3: Horizontal tracking wheel (THIS MUST BE NULL)
    nullptr,   // Slot 4: Second horizontal wheel
    &imu       // Slot 5: Inertial sensor
);

// drive curves
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 5, 1);

// chassis
lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
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
// function for motor movements and tongue mech.
void movements(bool tongue, int motor1, int motor2and3){          // REVIEW LOGIC FOR THIS FUNCTION!!!!!!!
    tonguePiston.set_value(tongue);
    n1.move_velocity(motor1);
    n2.move_velocity(motor2and3);
    n3.move_velocity(motor2and3);
}

// use when tuning the PID
void test(){
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);
}

// use when testing the auton path. Test one section before adding the next section
void leftAuton(){
    float autonMotorSpeedL = 80;
    float autonMoveSpeedL = 80;
    movements(true, 0, 0); // set everything to default (motors off, tongue mech up)

    // Go to center blocks and start intake
    chassis.moveToPose(-35, 22.5, 80, 3000, {.forwards = true, .maxSpeed = autonMoveSpeedL});
    //chassis.waitUntilDone();
    movements(true, autonMotorSpeedL, 0);
    
    /*
    // Move over the middle blocks
    chassis.moveToPose(-25, 22.5, 90, 3000, {.forwards = true, .maxSpeed = autonMoveSpeedL});
    chassis.waitUntilDone();
    pros::delay(1000); //wait for intake to be done
    movements(true, 0, 0);

    // Line up to the match loader
    chassis.moveToPose(-46, 47, 285, 5000, {.forwards = true, .maxSpeed = autonMoveSpeedL});
    chassis.waitUntilDone();
    movements(false, autonMotorSpeedL, 0);

    // Move to the match loader and intake blocks
    chassis.moveToPose(-56, 47, 270, 5000, {.forwards = true, .maxSpeed = autonMoveSpeedL});
    //might need to add shake function here
    pros::delay(2000); //wait for intake to be done

    // Move backward
    chassis.moveToPose(-40, 47, 270, 5000, {.forwards = false, .maxSpeed = autonMoveSpeedL});
    movements(true, 0, 0);

    // Move to the side goals and dispense
    chassis.moveToPose(-33, 47, 270, 5000, {.forwards = false, .maxSpeed = autonMoveSpeedL});
    chassis.waitUntilDone();
    movements(true, autonMotorSpeedL, -autonMotorSpeedL);
    */

}

void initialize() {
    pros::lcd::initialize();
    
    // 1. Physically zero out the rotation sensor hardware
    verticalEnc.reset_position(); 
    
    // 2. Calibrate the IMU
    chassis.calibrate();
    
    // 3. Set the starting position to 0,0,0
    // Without this, LemLib might interpret the first "degree" as a massive jump
    chassis.setPose(0, 0, 0); 

    tonguePiston.set_value(true); 

    pros::Task screenTask([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose();
            pros::lcd::print(0, "X: %.2f", pose.x);
            pros::lcd::print(1, "Y: %.2f", pose.y);
            pros::lcd::print(2, "Theta: %.2f", pose.theta);
            
            // This line helps you debug: 
            // If you push the bot 1 foot, this number should be roughly 18,000 to 20,000
            pros::lcd::print(3, "Raw Ticks: %d", verticalEnc.get_position());
            
            pros::delay(50);
        }
    });
}

void disabled() {}
void competition_initialize() {
    tonguePiston.set_value(true); // Tongue Up
}

// ==============================
// AUTONOMOUS
// ==============================
void autonomous() {
    test();
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
        }
        else if (r2) {
            // Manual Outtake via Button
            n1.move_velocity(MANUAL_OUTTAKE_SPEED);
            n2.move_velocity(-MANUAL_OUTTAKE_SPEED);
            n3.move_velocity(-MANUAL_OUTTAKE_SPEED);
        } 
        else if (std::abs(rightY) > INTAKEJOY_DEADZONE) {
            // Intake/Outtake via Joystick (Does NOT open the door)
            int speed = static_cast<int>(expo(rightY, INTAKE_EXPO_K));
            int rpm = (speed * MAX_INTAKE_RPM) / 127;

            n1.move_velocity(rpm);
            n2.move_velocity(rpm);
            n2.move_velocity(0);
        } 
        else {
            n1.move_velocity(0);
            n2.move_velocity(0);
            n3.move_velocity(0);
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
