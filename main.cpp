#include "main.h"
#include <cmath>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

//Global Variables

pros::Task* motorTask = nullptr;
bool motorsRunning = false;

// CONTROLLER
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// MOTOR GROUPS
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

// DRIVETRAIN
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    12,
    3.25, 
    450,
    2 
);

// controllers
lemlib::ControllerSettings lateral_controller(17, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              75, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              67, // derivative gain (kD)
                                              2.342, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew));
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


// function for motor movements and tongue mech.
void movements(bool tongue, bool door, int motor1, int motor2and3, int duration_ms = 0, bool unjam = false) {
    tonguePiston.set_value(tongue);
    doorPiston.set_value(door);

    // Stop any existing motor task first
    if (motorTask != nullptr) {
        motorsRunning = false;
        pros::delay(50); // Give task time to stop
        delete motorTask;
        motorTask = nullptr;
    }
    
    if (duration_ms > 0) {
        // Non-blocking mode with timed operation
        motorsRunning = true;
        
        motorTask = new pros::Task([motor1, motor2and3, duration_ms, unjam]() {
            unsigned long startTime = pros::millis();
            unsigned long lastUnjamTime = pros::millis();
            
            while (motorsRunning && (pros::millis() - startTime < duration_ms)) {
                if (unjam) {
                    unsigned long elapsed = pros::millis() - lastUnjamTime;
                    
                    // Run forward for 3 seconds
                    if (elapsed < 2000) {
                        n1.move(motor1);
                        n2.move(motor2and3);
                        n3.move(motor2and3);
                    }
                    // Run backward for 0.5 seconds
                    else if (elapsed < 2250) {
                        n1.move(-motor1);
                        n2.move(-motor2and3);
                        n3.move(-motor2and3);
                    }
                    // Reset cycle
                    else {
                        lastUnjamTime = pros::millis();
                    }
                } else {
                    // No unjamming, just run motors normally
                    n1.move(motor1);
                    n2.move(motor2and3);
                    n3.move(motor2and3);
                }
                
                pros::delay(10);
            }
            
            // Stop motors when done
            n1.move(0);
            n2.move(0);
            n3.move(0);
            motorsRunning = false;
        });
    } else {
        // Blocking mode (original behavior) - immediate control
        n1.move(motor1);
        n2.move(motor2and3);
        n3.move(motor2and3);
    }
}

// Stop motors early and clean up task
void stopMotors() {
    motorsRunning = false;
    
    if (motorTask != nullptr) {
        pros::delay(50); // Give task time to stop
        delete motorTask;
        motorTask = nullptr;
    }
    
    n1.move(0);
    n2.move(0);
    n3.move(0);
}



// Shake Tongue Mech
void shakeTongueMech(int cycles) {
    for (int i = 0; i < cycles; i++) {
        leftMotors.move(-120);
        rightMotors.move(-120);

        pros::delay(100); 
        leftMotors.move(0);
        rightMotors.move(0);
        pros::delay(300);

        
    }
}




// ==============================
// AUTON PATHS
// ==============================

// use when testing the auton path. Test one section before adding the next section



void initialize() {
    pros::lcd::initialize();
    
    // 1. Physically zero out the rotation sensor hardware
    verticalEnc.reset_position(); 
    
    // 2. Calibrate the IMU
    chassis.calibrate();
    
    // 3. Set the starting position to 0,0,0
    // Without this, LemLib might interpret the first "degree" as a massive jump
    chassis.setPose(0,0, 0); 

    tonguePiston.set_value(false);

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
    tonguePiston.set_value(false); // Tongue Up
}

// ==============================
// AUTONOMOUS
// ==============================

void leftAuton() {
    // Start values and speed
    int autoSpeed = 400;
    movements(false, true, 0, 0, 0, false);

    // Move to in front of the balls
    chassis.moveToPose(4, -28, -9, 1000, {.forwards = false});
    movements(false, true, autoSpeed, 0, 4000, false);

    chassis.moveToPoint(7,-45, 750, {.forwards = false, .maxSpeed = 100});
    chassis.turnToHeading(235,500);

    // Line up with the loader and turn towards it
    chassis.moveToPoint(32,-22, 1000, {.forwards = false,.maxSpeed = 80});
    chassis.turnToHeading(180, 750);

    // Go to the loader after being lined up and take blocks
    movements(true, true, autoSpeed, 0, 5000, false);
    chassis.moveToPoint(32,-7.5, 1500, {.forwards = false,.maxSpeed = 60});
    chassis.waitUntilDone();
    shakeTongueMech(6);
    chassis.waitUntilDone();



    chassis.moveToPoint(32,-36.5, 1000, {.forwards = true,.maxSpeed = 80});
    chassis.waitUntilDone();
    movements(false, true, autoSpeed, -autoSpeed, 4000, true);
    //chassis.moveToPoint(30,-40, 4000, {.forwards = true,.maxSpeed = 40});
}

void rightAuton() {
    // Start values and speed
    int autoSpeed = 400;
    movements(false, true, 0, 0, 0, false);

    // Move to in front of the balls
    chassis.moveToPose(-4, -28, 9, 1000, {.forwards = false});
    movements(false, true, autoSpeed, 0, 4000, false);

    chassis.moveToPoint(-7,-45, 750, {.forwards = false, .maxSpeed = 100});
    chassis.turnToHeading(235-90,500);

    // Line up with the loader and turn towards it
    chassis.moveToPoint(-32,-22, 1000, {.forwards = false,.maxSpeed = 80});
    chassis.turnToHeading(180, 750);

    // Go to the loader after being lined up and take blocks
    movements(true, true, autoSpeed, 0, 5000, false);
    chassis.moveToPoint(-32,-7.5, 1500, {.forwards = false,.maxSpeed = 60});
    chassis.waitUntilDone();
    shakeTongueMech(6);
    chassis.waitUntilDone();



    chassis.moveToPoint(-32,-36.5, 1000, {.forwards = true,.maxSpeed = 80});
    chassis.waitUntilDone();
    movements(false, true, autoSpeed, -autoSpeed, 4000, true);
    //chassis.moveToPoint(30,-40, 4000, {.forwards = true,.maxSpeed = 40});
}

void autonomous() {
    leftAuton();
}

// DRIVER CONTROL
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
            //
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
