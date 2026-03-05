#include "main.h"
#include <cmath>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

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
                                              1, // integral gain (kI)
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
lemlib::ExpoDriveCurve steerCurve(5, 5, 1);

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
// AUTON HELPERS
// ==============================

// function for motor movements and tongue mech.
void movements(bool tongue, bool door, int motor1, int motor2and3){          // REVIEW LOGIC FOR THIS FUNCTION!!!!!!!
    tonguePiston.set_value(tongue);
    doorPiston.set_value(door);
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

// Shake Tongue Mech for Skills
void shakeTongueMechSkills(int x,int y, int goal){
    for(int i = 0; i < 3; i++) {
        chassis.moveToPose(x*60, y*47, goal, 500);
        pros::delay(500);
    }
}

// Shake Tongue Mech according to position
void shakeTongueMechPos(int x,int y, int goal){
    for(int i = 0; i < 3; i++) {
        chassis.moveToPose(x, y, goal, 500);
        pros::delay(500);
    }
}

// Decide which goal is being emptied, assign according values, and then empty the goal
void outtakeAndGoal(int corner, int autonMotorSpeedS, float autonMoveSpeedS){
    int x,y,face,goal;
    if(corner == 0){
        x = -1;
        y = -1;
        face = 180;
        goal = 270;
    } else if(corner == 1){
        x = -1;
        y = 1;
        face = 0;
        goal = 270;
    } else if(corner == 2){
        x = 1;
        y = 1;
        face = 40;
        goal = 90;
    } else if(corner == 3){
        x = 1;
        y = -1;
        face = 180;
        goal = 90;    
    }
    
    // 1. Line up with the goal 
    chassis.moveToPose(x*44, y*47, face, 6000, {.forwards = false, .maxSpeed = autonMoveSpeedS});
    movements(false, true, autonMotorSpeedS, -autonMotorSpeedS);
    
    // 2. Go to the goal
    chassis.moveToPose(x*57, y*47, goal, 6000, {.forwards = false, .maxSpeed = autonMoveSpeedS});
    shakeTongueMechSkills(x,y,goal);

    // 3. Leave the goal
    chassis.moveToPose(x*44, y*47, goal, 6000, {.forwards = true, .maxSpeed = autonMoveSpeedS});
    movements(true, true, 0, 0);
}

// ==============================
// AUTON PATHS
// ==============================

// use when testing the auton path. Test one section before adding the next section
void leftAuton(){
    chassis.setPose(-56,-16.75,90);
    float autonMotorSpeedL = 100;
    float autonMoveSpeedL = 100;
    movements(false, true, 0, 0); // set everything to default (motors off, tongue mech up)

    // Go to center blocks and start intake
    chassis.moveToPose(-35, -22.5, 90, +2000, {.forwards = true, .maxSpeed = autonMoveSpeedL});
    //movements(true, true, autonMotorSpeedL, 0);

    // Turn to 90 degrees
    chassis.turnToHeading(75, 1000);

    // Move over the middle blocks
    chassis.moveToPoint(-15, -22.5, 3000, {.forwards = true, .maxSpeed = autonMoveSpeedL});
    //pros::delay(1000); //wait for intake to be done
    //(true, 0, 0);

    chassis.moveToPoint(4, 6, 1000);
    // Turn to 310 degrees
    chassis.turnToHeading(310, 1000);

    // Line up to the match loader
    chassis.moveToPoint(-46, -47, 5000, {    .forwards = true, .maxSpeed = autonMoveSpeedL});
    //movements(false, true, autonMotorSpeedL, 0);

    // Turn to 270 degrees
    chassis.turnToHeading(270, 1000);

    // Move to the match loader and intake blocks
    chassis.moveToPoint(-56, -47, 5000, {.forwards = true, .maxSpeed = autonMoveSpeedL});
    //shakeTongueMechPos(-72, 47, 270);
    //movements(true, true, 0, 0);

    // Move to the side goals and dispense
    chassis.moveToPoint(-33, -47, 5000, {.forwards = false, .maxSpeed = autonMoveSpeedL});
    //movements(true, true, autonMotorSpeedL, -autonMotorSpeedL);
}   

void skillsAuton(){
    //start -61,-16, 90
    float autonMotorSpeedS = 50;
    float autonMoveSpeedS = 80;
    movements(true, true, 0, 0);
                
    // 1. Get away from the parking zone
    chassis.moveToPose(-44, -16, 90, 2000, {.forwards = false, .maxSpeed = autonMoveSpeedS});

    // 2. Empty the bottom left goal
    outtakeAndGoal(0, autonMotorSpeedS, -autonMoveSpeedS);

    /*
    // 3. Empty the top left goal
    outtakeAndGoal(1, autonMotorSpeedS, autonMoveSpeedS);

    // 4. Go to the center
    chassis.moveToPose(-25, 25, 140, 6000, {.forwards = true, .maxSpeed = autonMoveSpeedS});
    chassis.moveToPose(25, 25, 90, 6000, {.forwards = true, .maxSpeed = autonMoveSpeedS});

    // 5. Empty the top right goal
    outtakeAndGoal(2, autonMotorSpeedS, autonMoveSpeedS);

    // 6. Empty the bottom right goal
    outtakeAndGoal(3, autonMotorSpeedS, autonMoveSpeedS);

    // 7. Go to the center
    chassis.moveToPose(25, -25, 320, 6000, {.forwards = true, .maxSpeed = autonMoveSpeedS});
    chassis.moveToPose(-25, -25, 270, 6000, {.forwards = true, .maxSpeed = autonMoveSpeedS});

    // 8. Line up with the parking spot
    chassis.moveToPose(-66, 3, 310, 6000, {.forwards = true, .maxSpeed = 127});
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
    chassis.setPose(0,0, 0); 

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
    // Start values and speed
    int autoSpeed = 400;
    movements(false, true, 0, 0);

    // Move to in front of the balls
    chassis.moveToPose(4, -28, -9, 2000, {.forwards = false});
    movements(false, true, autoSpeed, 0);

    // Move over the balls and turn
    
    movements(false, true, 0, 0);
    chassis.moveToPoint(7,-45, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.turnToHeading(145+90,1000);

    // Line up with the loader and turn towards it
    chassis.moveToPoint(30,-22, 2000, {.forwards = false});
    chassis.turnToHeading(180, 1000);

    // Go to the loader after being lined up and take blocks
    movements(true, true, autoSpeed, 0);
    chassis.moveToPoint(32,10, 2000, {.forwards = false});
    movements(true, true, 0, 0);
    // shake time
    // Go backward to the goal
    chassis.moveToPoint(32,-35.5, 2000, {.forwards = true,.maxSpeed = 80});
    movements(false, true, autoSpeed, -autoSpeed);
}


/* Old code
// Move to in front of the balls
    chassis.moveToPose(4, -28, -9, 2000, {.forwards = false});// change
    movements(false, 100, 0);

    // Move over the balls and turn 
    movements(false, 100, 0);
    chassis.moveToPoint(7,-45, 2000, {.forwards = false});// change
    chassis.turnToHeading(145,1000);// change

    // Line up with the loader and turn towards it
    chassis.moveToPoint(30,-22, 2000, {.forwards = false});// change
    chassis.turnToHeading(180,1000);

    // Go to the loader after being lined up and take blocks
    movements(true, 0, 0);
    pros::delay(1000);
    movements(true, 100, 0);

    chassis.moveToPoint(32,0, 750, {.forwards = false});// change to forward of the loader
    chassis.moveToPoint(32,-10.5, 2000, {.forwards = false});
    // shake time
    // Go backward to the goal
    chassis.moveToPoint(32,-35.5, 2000, {.forwards = true});
    movements(false, 100, -100);
*/

/* Super Old code (works)
chassis.moveToPoint(0,-20.5, 2000, {.forwards = false});
    chassis.turnToHeading(270,1000);
    chassis.moveToPoint(32,-20.5, 2000, {.forwards = false});
    chassis.turnToHeading(180,1000);
    chassis.moveToPoint(32,-10.5, 2000, {.forwards = false});
    // shake time
    chassis.moveToPoint(32,-35.5, 2000, {.forwards = true});
*/

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
