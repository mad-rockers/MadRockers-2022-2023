#include "Robot.h"

void Robot::drivetrain() {
    float speed;
    if(r_driver_right.GetTrigger()) {
        speed = 0.5;
    }
    else {
        speed = 1;
    }

    float left_power, right_power;

    if(r_left_front.GetInverted()) {
        left_power = r_driver_right.GetY() * speed;
        right_power = r_driver_left.GetY() * speed;
    }
    else {
        left_power = r_driver_left.GetY() * speed;
        right_power = r_driver_right.GetY() * speed;
    }
    
    r_drivetrain.TankDrive(left_power, right_power, false);

    if(r_driver_left.GetTriggerPressed()) {
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
    if(r_operator.GetRightBumper()) {
        r_box.Set(true);
    }
    if(r_operator.GetLeftBumper()) {
        r_box.Set(false);
    }
}

void Robot::grabber() {
    if(r_operator.GetAButton()) {
        r_grabber.Set(true);
    }
    if(r_operator.GetYButton()) {
        r_grabber.Set(false);
    }
}