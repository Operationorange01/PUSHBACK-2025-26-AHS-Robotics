#include "main.h" 
#include <cmath>
#include <string> 
#include <vector>
#include "lemlib/api.hpp" // IWYU pragma: keep

#ifdef __cplusplus
typedef int lcd_buttons_t; 
#else 
typedef int pros::lcd_buttons_t; 
#endif

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

// function declarations
void isSkills();
void isCarry();
void whichAuton();
void whichCarry();
void reset();

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

const std::vector<std::string> skillsOptions = {"Skills Auton", "Competition Auton"};
const std::vector<std::string> carryOptions = {"Carry Teammate", "Solo Auton", "Back"};
const std::vector<std::string> whichCarryOptions = {"Left Carry", "Right Carry", "Back"};
const std::vector<std::string> whichAutonOptions = {"Left Side", "Right Side", "Left Middle", "Right Middle", "Back"};

// gets buttons pressed
int getButtons() {
    
    // read the currently pressed buttons
    int pressedButtons = pros::lcd::read_buttons();
    
    if (pressedButtons != 0) {
        // wait until buttons are released
        while (pros::lcd::read_buttons() != 0) {
            pros::delay(10);
        }
    }
    return pressedButtons;
}

// draw the menu and handle selection
void drawMenu(const std::string& title, const std::vector<std::string>& options, int currentSelection) {
    pros::lcd::clear();
    // print the title
    pros::lcd::set_text(0, title);
    
    // number of options
    int numOptions = options.size(); 

    for (int i = 0; i < numOptions; i++) {
        // build display string
        std::string displayText;
        
        // cool selection indicator
        if (i == currentSelection) {
            displayText = ">> " + options[i] + " <<";
        } else {
            displayText = "   " + options[i] + "   ";
        }
        
        // print the string on line i + 1
        pros::lcd::print(i + 1, displayText.c_str());
    }
}

// checks if it is a skills match or a normal match and runs the correct auton
void isSkills(){
    int selection = 0;
    int numOptions = skillsOptions.size(); 

    while(true){
        drawMenu("Match Type", skillsOptions, selection);
        
        auto buttons = getButtons();
        
        // moves up the selection
        if (buttons & LCD_BTN_LEFT) { 
            selection = (selection - 1 + numOptions) % numOptions;
        } 
        // moves down the selection
        else if (buttons & LCD_BTN_RIGHT) { 
            selection = (selection + 1) % numOptions;
        } 
        // selects the selection
        else if (buttons & LCD_BTN_CENTER) {
            if (selection == 0) { // skills auton
                skillsAuton();
                break;
            } else { // no skills)
                isCarry();
                break;
            }
        }
        pros::delay(10);
    }
}

// checks if the auton needs to carry the teammate's bot or not
void isCarry(){
    int selection = 0;
    int numOptions = carryOptions.size(); 

    while(true){
        drawMenu("Carry Status", carryOptions, selection);

        auto buttons = getButtons();

        // moves up the selection
        if (buttons & LCD_BTN_LEFT) { 
            selection = (selection - 1 + numOptions) % numOptions;
        } 
        // moves down the selection
        else if (buttons & LCD_BTN_RIGHT) { 
            selection = (selection + 1) % numOptions;
        } 
        // selects the selection
        else if (buttons & LCD_BTN_CENTER) {
            if (selection == 0) { // Carry Teammate
                whichCarry();
                break;
            } else if(selection == 1){ // Solo Auton (No Carry)
                whichAuton();
                break;
            } else if(selection == 2){ // Back
                isSkills();
                return;
            }
        }
        pros::delay(10);
    }
}

// checks which side to start on for normal auton and whether to go to the middle or not
void whichAuton(){
    int selection = 0;
    int numOptions = whichAutonOptions.size(); 

    while(true){
        drawMenu("Solo Auton", whichAutonOptions, selection);

        auto buttons = getButtons();
        
        // moves up the selection
        if (buttons & LCD_BTN_LEFT) { 
            selection = (selection - 1 + numOptions) % numOptions;
        } 
        // moves down the selection
        else if (buttons & LCD_BTN_RIGHT) { 
            selection = (selection + 1) % numOptions;
        } 
        // selects the selection
        else if (buttons & LCD_BTN_CENTER) {
            switch(selection) {
                case 0: leftAuton(); break;
                case 1: rightAuton(); break;
                case 2: leftMiddleAuton(); break;
                case 3: rightMiddleAuton(); break;
                case 4: isCarry(); return; // Back
            }
            if (selection != 4) break;
        }
        pros::delay(10);
    }
}

// checks which side to start on for the carry auton
void whichCarry(){
    int selection = 0;
    int numOptions = whichCarryOptions.size(); 

    while(true){
        drawMenu("Carry Auton", whichCarryOptions, selection);

        auto buttons = getButtons();
        
        // moves up the selection
        if (buttons & LCD_BTN_LEFT) { 
            selection = (selection - 1 + numOptions) % numOptions;
        } 
        // moves down the selection
        else if (buttons & LCD_BTN_RIGHT) { 
            selection = (selection + 1) % numOptions;
        } 
        // selects the selection
        else if (buttons & LCD_BTN_CENTER) {
            switch(selection) {
                case 0: leftCarryAuton(); break;
                case 1: rightCarryAuton(); break;
                case 2: isCarry(); return; // Back
            }
            if (selection != 2) break;
        }
        pros::delay(10);
    }
}

// resets to the first auton choice screen (i didn't use this function and idk if it works, but wtv)
void reset(){
    pros::lcd::clear();
    pros::lcd::print(0, "Resetting choices");
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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    isSkills(); // start auton selection menu, it is temporarily here for testing. it should be only in autonomous() during the competition.

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

// exponential function for intake joystick
double expo(int input, double k) {
    double x = input / 127.0;
    return (k * x * x * x + (1.0 - k) * x) * 127.0;
}

// runs during operator control 
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
