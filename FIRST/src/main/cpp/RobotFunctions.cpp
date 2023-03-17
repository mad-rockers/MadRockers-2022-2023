#include "Robot.h"

void Robot::drivetrain() {
    float speed;
    if(r_driver.GetRightBumper()) {
        speed = 0.5;
    }
    else {
        speed = 1;
    }

    float left_power, right_power;

    if(r_left_front.GetInverted()) {
        left_power = r_driver.GetLeftY() * speed;
        right_power = r_driver.GetRightY() * speed;
    }
    else {
        left_power = r_driver.GetRightY() * speed;
        right_power = r_driver.GetLeftY() * speed;
    }
    
    r_drivetrain.TankDrive(left_power, right_power, false);

    if(r_driver.GetAButton()) {
        r_left_front.SetInverted(!r_left_front.GetInverted());
        r_right_front.SetInverted(!r_right_front.GetInverted());
        while(r_driver.GetAButton()) {}
    }
}

void Robot::arm() {
    r_arm.Set(r_operator.GetRightY());
    if(r_arm_limit_high.Get() && r_arm.Get() < 0) {
        r_arm.Set(0);
    }
    if(r_arm_limit_low.Get() && r_arm.Get() > 0) {
        r_arm.Set(0);
    }
}

void Robot::extension() {
    r_extension.Set(r_operator.GetLeftY());
    if(r_extension_limit_front.Get() && r_extension.Get() < 0) {
        r_extension.Set(0);
    }
    if(r_extension_limit_back.Get() && r_extension.Get() > 0) {
        r_extension.Set(0);
    }
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
        r_low_grabber.Set(true);
        r_high_grabber.Set(false);
    }
    if(r_operator.GetBButton()) {
        r_low_grabber.Set(true);
        r_high_grabber.Set(true);
    }
    if(r_operator.GetYButton()) {
        r_low_grabber.Set(false);
        r_high_grabber.Set(false);
    }
}