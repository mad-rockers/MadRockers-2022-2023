#include "Robot.h"

void Robot::drivetrain() {
    float speed;
    if(r_driver.GetR1Button()) {
        speed = 0.5;
    }
    else {
        speed = 1;
    }

    float left_power, right_power;

    if(r_left_front.GetInverted()) {
        left_power = r_driver.GetRightY() * speed;
        right_power = r_driver.GetLeftY() * speed;
    }
    else {
        left_power = r_driver.GetLeftY() * speed;
        right_power = r_driver.GetRightY() * speed;
    }
    
    r_drivetrain.TankDrive(left_power, right_power, false);

    if(r_driver.GetCrossButton()) {
        r_left_front.SetInverted(!r_left_front.GetInverted());
        r_right_front.SetInverted(!r_right_front.GetInverted());
    }
}

void Robot::arm() {
    r_arm.Set(r_operator.GetRightY());
}

void Robot::extension() {
    r_extension.Set(r_operator.GetLeftY());
}

void Robot::box() {
    if(r_operator.GetR2Axis()) {
        r_box.Set(true);
    }
    if(r_operator.GetL2Axis()) {
        r_box.Set(false);
    }
}

void Robot::grabber() {
    if(r_operator.GetCrossButton()) {
        r_high_grabber.Set(true);
    }
    if(r_operator.GetTriangleButton()) {
        r_high_grabber.Set(false);
    }
}