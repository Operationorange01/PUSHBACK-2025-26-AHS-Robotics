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

pros::ADIDigitalOut outtakePiston('A');

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
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
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
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

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
// all of the paths needed for the autons should be defined here
ASSET(path11_txt); // '.' replaced with "_" to make c++ happy
ASSET(path12_txt);
ASSET(path13_txt);
ASSET(path21_txt)
ASSET(path22_txt);
ASSET(path23_txt);
ASSET(path31_txt);
ASSET(path32_txt);
ASSET(path33_txt);
ASSET(path34_txt);
ASSET(path41_txt);
ASSET(path42_txt);
ASSET(path43_txt);
ASSET(path44_txt);

// all of the functions for the autons

// needed when teammate has a bad auton and picks the left side
void leftCarryAuton(){
    pros::lcd::clear();
    pros::lcd::print(0, "Running Left Carry Auton");
    //insert code here
}

// needed when teammate has a bad auton and picks the right side
void rightCarryAuton(){
    pros::lcd::clear();
    pros::lcd::print(0, "Running Right Carry Auton");
    //insert code here
}

// auton that starts on the left side
void leftAuton(){
    pros::lcd::clear();
    pros::lcd::print(0, "Running Left Auton");
    //insert code here
}

// auton that starts on the right side
void rightAuton(){
    pros::lcd::clear();
    pros::lcd::print(0, "Running Right Auton");
    //insert code here
}

// auton that starts on the left and goes to the middle before continuing
void leftMiddleAuton(){
    pros::lcd::clear();
    pros::lcd::print(0, "Running Left Middle Auton");
    //insert code here
}

// auton that starts on the right and goes to the middle before continuing
void rightMiddleAuton(){
    pros::lcd::clear();
    pros::lcd::print(0, "Running Right Middle Auton");
    //insert code here
}

// skills auton
void skillsAuton(){
    pros::lcd::clear();
    pros::lcd::print(0, "Running Skills Auton");
    //insert code here
}

// checks if it is a skills match or a normal match and runs the correct auton
bool isTouching(int rectX, int rectY, int rectWidth, int rectHeight, int touchX, int touchY){
    return touchX >= rectX && touchX <= (rectX + rectWidth) && 
            touchY >= rectY && touchY <= (rectY + rectHeight);
}

// checks if it is a skills match or a normal match and runs the correct auton
void isSkills(){
    pros::lcd::clear();

    //  define button positions and sizes
    int skillsX = 40, skillsY = 50, skillsWidth = 180,skillsHeight = 100;
    int noSkillsX = 220, noSkillsY = 50, noSkillsWidth = 180, noSkillsHeight = 100;

    // skills button
    pros::lcd::draw_rectangle(skillsX, skillsY, skillsWidth, skillsHeight);
    pros::lcd::print_at(skillsX + 10, skillsY + 10, "Skills");

    // no skills button
    pros::lcd::draw_rectangle(noSkillsX, noSkillsY, noSkillsWidth, noSkillsHeight);
    pros::lcd::print_at(noSkillsX + 10, noSkillsY + 10, "No Skills");
    
    // check if the button is pressed and which one
    while(true){
        //  get touch point
        if(pros::lcd::is_touching()){
            pros::lcd::touch_point_t touch = pros::lcd::get_touch_point();
            int x = touch.x;
            int y = touch.y;

            // check if the skills button is pressed
            if(isTouching(skillsX, skillsY, skillsWidth, skillsHeight, x, y)){
                // run skills auton
                skillsAuton();
                break;
            }
            // check if the no skills button is pressed
            else if(isTouching(noSkillsX, noSkillsY, noSkillsWidth, noSkillsHeight, x, y)){
                // run isCarry to determine if the auton needs to carry the teammate or not
                isCarry()
                break;
            }
        }
    }
}

// checks if the auton needs to carry the teammate's bot or not
void isCarry(){
    pros::lcd::clear();

    //  define button positions and sizes
    int carryX = 40, carryY = 50, carryWidth = 180, carryHeight = 100;
    int noCarryX = 220, noCarryY = 50, noCarryWidth = 180, noCarryHeight = 100;
    int resetX = 130, resetY = 200, resetWidth = 180, resetHeight = 100;

    // carry button
    pros::lcd::draw_rectangle(carryX, carryY, carryWidth, carryHeight);
    pros::lcd::print_at(carryX + 10, carryY + 10, "Carry");

    // no carry button
    pros::lcd::draw_rectangle(noCarryX, noCarryY, noCarryWidth, noCarryHeight);
    pros::lcd::print_at(noCarryX + 10, noCarryY + 10, "No Carry");

    // no skills button
    pros::lcd::draw_rectangle(resetX, resetY, resetWidth, resetHeight);
    pros::lcd::print_at(resetX + 10, resetY + 10, "Reset");

    // check if the button is pressed and which one
    while(true){
        //  get touch point
        if(pros::lcd::is_touching()){
            pros::lcd::touch_point_t touch = pros::lcd::get_touch_point();
            int x = touch.x;
            int y = touch.y;

            // check if the carry button is pressed
            if(isTouching(carryX, carryY, carryWidth, carryHeight, x, y)){
                skillsAuton();
                break;
            }
            // check if the no carry button is pressed
            else if(isTouching(noCarryX, noCarryY, noCarryWidth, noCarryHeight, x, y)){
                // run whichCarry to determine which side to start on
                whichCarry();
                break;
            } else if(isTouching(resetX, resetY, resetWidth, resetHeight, x, y)){
                // go back to isSkills
                reset();
                break;
            }
        }
    }
}

// checks which side to start on for normal auton and whether to go to the middle or not
void whichAuton(){
    pros::lcd::clear();

    //  define button positions and sizes
    int topLeftX = 40, topLeftY = 20, topLeftWidth = 180, topLeftHeight = 100;
    int topRightX = 220, topRightY = 20, topRightWidth = 180, topRightHeight = 100;
    int bottomLeftX = 40, bottomLeftY = 150, bottomLeftWidth = 180, bottomLeftHeight = 100;
    int bottomRightX = 220, bottomRightY = 150, bottomRightWidth = 180, bottomRightHeight = 100;
    int resetX = 200, resetY = 168, resetWidth = 180, resetHeight = 100;

    // carry button
    pros::lcd::draw_rectangle(topLeftX, topLeftY, topLeftWidth, topLeftHeight);
    pros::lcd::print_at(topLeftX + 10, topLeftY + 10, "Left");

    // no carry button
    pros::lcd::draw_rectangle(topRightX, topRightY, topRightWidth, topRightHeight);
    pros::lcd::print_at(topRightX + 10, topRightY + 10, "Right");

    // no skills button
    pros::lcd::draw_rectangle(bottomLeftX, bottomLeftY, bottomLeftWidth, bottomLeftHeight);
    pros::lcd::print_at(bottomLeftX + 10, bottomLeftY + 10, "Left Middle");

    // no skills button
    pros::lcd::draw_rectangle(bottomRightX, bottomRightY, bottomRightWidth, bottomRightHeight);
    pros::lcd::print_at(bottomRightX + 10, bottomRightY + 10, "Right Middle");
    // no skills button
    pros::lcd::draw_rectangle(resetX, resetY, resetWidth, resetHeight);
    pros::lcd::print_at(resetX + 10, resetY + 10, "Reset");

    // check if the button is pressed and which one
    while(true){
        //  get touch point
        if(pros::lcd::is_touching()){
            pros::lcd::touch_point_t touch = pros::lcd::get_touch_point();
            int x = touch.x;
            int y = touch.y;

            // check if the left button is pressed
            if(isTouching(topLeftX, topLeftY, topLeftWidth, topLeftHeight, x, y)){
                leftAuton();
                break;
            // check if the right button is pressed
            } else if(isTouching(topRightX, topRightY, topRightWidth, topRightHeight, x, y)){
                rightAuton();
                break;
            // check if the left middle button is pressed
            } else if(isTouching(bottomLeftX, bottomLeftY, bottomLeftWidth, bottomLeftHeight, x, y)){
                leftMiddleAuton();
                break;
            //check if the right middle button is pressed
            } else if(isTouching(bottomRightX, bottomRightY, bottomRightWidth, bottomRightHeight, x, y)){
                rightMiddleAuton();
                break;
            // check if the reset button is pressed
            } else if(isTouching(resetX, resetY, resetWidth, resetHeight, x, y)){
                // go back to isSkills
                reset();
                break;
            }
        }
    }
}

// checks which side to start on for the carry auton
void whichCarry(){
    pros::lcd::clear();

    //  define button positions and sizes
    int leftCarryX = 40, leftCarryY = 50, leftCarryWidth = 180, leftCarryHeight = 100;
    int rightCarryX = 220, rightCarryY = 50, rightCarryWidth = 180, rightCarryHeight = 100;
    int resetX = 130, resetY = 200, resetWidth = 180, resetHeight = 100;

    // carry button
    pros::lcd::draw_rectangle(leftCarryX, leftCarryY, leftCarryWidth, leftCarryHeight);
    pros::lcd::print_at(leftCarryX + 10, leftCarryY + 10, "Left Carry");

    // no carry button
    pros::lcd::draw_rectangle(rightCarryX, rightCarryY, rightCarryWidth, rightCarryHeight);
    pros::lcd::print_at(rightCarryX + 10, rightCarryY + 10, "Right Carry");

    // no skills button
    pros::lcd::draw_rectangle(resetX, resetY, resetWidth, resetHeight);
    pros::lcd::print_at(resetX + 10, resetY + 10, "Reset");

    // check if the button is pressed and which one
    while(true){
        //  get touch point
        if(pros::lcd::is_touching()){
            pros::lcd::touch_point_t touch = pros::lcd::get_touch_point();
            int x = touch.x;
            int y = touch.y;

            // check if the left carry button is pressed
            if(isTouching(leftCarryX, leftCarryY, leftCarryWidth, leftCarryHeight, x, y)){
                leftCarryAuton();
                break;
            }
            // check if the right carry button is pressed
            else if(isTouching(rightCarryX, rightCarryY, rightCarryWidth, rightCarryHeight, x, y)){
                rightCarryAuton();
                break;
            }
            // check if the reset button is pressed
            else if(isTouching(resetX, resetY, resetWidth, resetHeight, x, y)){
                // go back to isSkills
                reset();
                break;
            }
        }
    }
}

// resets to the initial auton choice screen
void reset(){
    pros::lcd::clear();
    pros::lcd::print(0, "Resetting choices...");
    isSkills();
}

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    isSkills();
    
	/*
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");*/
}

/**
 * Runs in driver control
 */

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

        // CURVATURE DRIVE (optional)
        // chassis.curvature(leftY, leftX, false);
        // - leftY: forward/backward input (-127 to 127)
        // - leftX: turn input (-127 to 127)
        // - third parameter: allowQuickTurn (true = robot can spin in place when forward input is near 0)
        // Example with quick turn enabled:
        // chassis.curvature(leftY, leftX, true);
		
		/*
        // Curvature drive explanation:
        // chassis.curvature(forward, turn);
        // - forward: controls the robot's speed forward/backward
        // - turn: controls turning
        // Difference from arcade drive:
        //   - Curvature drive scales turning based on forward speed.
        //     At high speed, turning is smoother and less twitchy.
        //     At low speed, turning is sharper for tight maneuvers.
        //   - Arcade drive (what we're currently using) mixes forward and turn linearly:
        //     leftPower  = forward + turn
        //     rightPower = forward - turn
        //   - Curvature drive is preferred for "car-like" feel and smoother high-speed driving.
        */

        
        // INTAKE CONTROL Joystick + Buttons
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        //Deadzone for intake joystick
        constexpr int INTAKEJOY_DEADZONE = 15;

        //Exponential factor for intake joystick
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

        pros::delay(10);
    }
}
