// move forward while intaking blocks
void autonDriveAndIntake(double targetX, double targetY, int timeoutMs, int intakeSpeed = 200) {
    // start intake motors
    n1.move_velocity(intakeSpeed);
    n2.move_velocity(intakeSpeed);
    n3.move_velocity(intakeSpeed);
    n4.move_velocity(0); // outtake stays off

    // drive to the target while maintaining current heading
    chassis.moveToPose(targetX, targetY, chassis.getPose().theta, timeoutMs);
    chassis.waitUntilDone();

    // stop intake motors
    n1.move_velocity(0);
    n2.move_velocity(0);
    n3.move_velocity(0);
}

// drive to a target position, automatically turning to face it first
void autonMoveToPosition(double targetX, double targetY, int timeoutMs, int intakeSpeed = 0) {
    // compute heading towards the target position
    double dx = targetX - chassis.getPose().x;
    double dy = targetY - chassis.getPose().y;
    double headingDeg = atan2(dy, dx) * 180.0 / M_PI;

    // optional intake while moving
    if (intakeSpeed > 0) {
        n1.move_velocity(intakeSpeed);
        n2.move_velocity(intakeSpeed);
        n3.move_velocity(intakeSpeed);
        n4.move_velocity(0);
    }

    // turn to face the target
    chassis.turnTo(headingDeg);
    chassis.waitUntilDone();

    // drive to the target
    chassis.moveToPose(targetX, targetY, headingDeg, timeoutMs);
    chassis.waitUntilDone();

    // stop intake if it was running
    if (intakeSpeed > 0) {
        n1.move_velocity(0);
        n2.move_velocity(0);
        n3.move_velocity(0);
    }
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
    autonDriveAndIntake(-76, -59, 3000, 200);

    // drive to next block line, automatically facing it
    autonMoveToPosition(-76.743, -121.431, 4000);

    // drive to long goal to deposit, automatically facing it
    autonMoveToPosition(-61.261, -121.431, 3000);

    // outtake all blocks
    autonOuttake(200);
}
