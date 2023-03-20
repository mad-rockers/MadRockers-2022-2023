#include "Robot.h"

void Robot::drivetrain() {
    float speed;
    if(r_driver.GetLeftBumper()) {
        if(timer.Get() == 0_s) {
            timer.Start();
            speed = 0.6;
        }
        else if(timer.Get() > 0.5_s) {
            speed = 1;
        }
    }
    else {
        timer.Stop();
        timer.Reset();
        speed = 0.3;
    }

    float left_power, right_power;
    left_power = r_driver.GetLeftY() * speed;
    right_power = r_driver.GetRightY() * speed;

    r_drivetrain.TankDrive(left_power, right_power, false);
}

void Robot::arm() {
    /*
    Arm States:
    0 - Normal control
    1 - Zero position
    2 - Grabbing position
    */
    double grab = -1;
    
    if(r_operator.GetYButton()) {
        arm_state = 1;
    }
    else if (r_operator.GetAButton()) {
        arm_state = 2;
    }
    else {
        arm_state = 0;
    }

    if(arm_state == 1) {
        r_arm_pid.SetReference(0, ControlType::kPosition);
    }
    else if(arm_state == 2) {
        r_arm_pid.SetReference(grab, ControlType::kPosition);
    }
    else {
        double setpoint = r_operator.GetLeftY();
        if(r_arm_limit_high.Get() && setpoint < 0) {
            r_arm.Set(0);
        }
        else if(r_arm_limit_low.Get() && setpoint > 0) {
            r_arm.Set(0);
        }
        else if(setpoint == 0) {
            r_arm_pid.SetReference(0, ControlType::kVelocity);
        }
        else {
            r_arm.Set(setpoint);
        }
    }
}

void Robot::extension() {
    /*
    Extension States:
    0 - Normal control
    1 - Zero position
    2 - Grabbing position
    */
    double grab = -46;

    if(r_operator.GetYButton()) {
        extension_state = 1;
    }
    else if (r_operator.GetAButton()) {
        extension_state = 2;
    }
    else {
        extension_state = 0;
    }

    if(extension_state == 1) {
        r_extension_pid.SetReference(0, ControlType::kPosition);
    }
    else if(extension_state == 2) {
        r_extension_pid.SetReference(grab, ControlType::kPosition);
    }
    else {
        double setpoint = r_operator.GetRightY();
        if(r_extension_limit_front.Get() && setpoint < 0) {
            r_extension.Set(0);
        }
        else if(r_extension_limit_back.Get() && setpoint > 0) {
            r_extension.Set(0);
        }
        else {
            r_extension.Set(setpoint);
        }
    }
}

void Robot::box() {
    if(r_driver.GetRightBumper()) {
        r_box.Set(true);
    }
    else {
        r_box.Set(false);
    }
}

void Robot::grabber() {
    if(r_operator.GetLeftBumper()) {
        grabber_open();
    }
    else if(r_operator.GetRightBumper()) {
        grabber_close_low();
    }
    else if(r_operator.GetRightTriggerAxis()) {
        grabber_close_high();
    }
}

void Robot::grabber_open() {
    r_low_grabber.Set(true);
    r_high_grabber.Set(false);
}

void Robot::grabber_close_low() {
    r_low_grabber.Set(false);
    r_high_grabber.Set(false);
}

void Robot::grabber_close_high() {
    r_low_grabber.Set(false);
    r_high_grabber.Set(true);
}