#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::drivetrain() {
    float slow_speed = 0.3;
    units::second_t accel_time = 1_s;
    float max_speed = 1;

    float speed;
    if(r_driver.GetLeftBumper()) {
        if(timer.Get() == 0_s) {
            timer.Start();
        }
        else if(timer.Get() < accel_time) {
            speed = double(timer.Get()) / double(accel_time) * (max_speed - slow_speed) + slow_speed;
        }
        else {
            speed = 1;
        }
    }
    else {
        timer.Stop();
        timer.Reset();
        speed = slow_speed;
    }

    float left_power, right_power;
    left_power = r_driver.GetLeftY() * speed;
    right_power = r_driver.GetRightY() * speed;

    if(left_power == 0 && right_power == 0 && r_driver.GetRightTriggerAxis()) {
        if(left_hold == 0) {
            left_hold = r_left_encoder.GetPosition();
            right_hold = r_right_encoder.GetPosition();
        }
        r_left_pid.SetReference(left_hold, ControlType::kPosition);
        r_right_pid.SetReference(right_hold, ControlType::kPosition);
    }
    else {
        left_hold = 0;
        r_left_front.Set(left_power);
        r_right_front.Set(right_power);
    }

    if(r_driver.GetAButton()) {
        while(r_driver.GetAButton()) {}
        r_left_front.SetInverted(!r_left_front.GetInverted());
        r_right_front.SetInverted(!r_right_front.GetInverted());
    }
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
    else if(r_operator.GetAButton()) {
        arm_state = 2;
    }
    else if(r_operator.GetLeftY() != 0) {
        arm_state = 0;
    }

    if(arm_state == 1) {
        r_arm_pid.SetReference(0, ControlType::kPosition);
    }
    else if(arm_state == 2) {
        r_arm_pid.SetReference(grab, ControlType::kPosition);
    }
    else {
        double setpoint = r_operator.GetLeftY() * 0.5;
        if(r_arm_limit_high.Get() && setpoint < 0) {
            r_arm.Set(0);
        }
        else if(r_arm_limit_low.Get() && setpoint > 0) {
            r_arm.Set(0);
        }
        else if(setpoint == 0) {
            if(arm_hold == 0) {
                arm_hold = r_arm_encoder.GetPosition();
            }
            r_arm_pid.SetReference(arm_hold, ControlType::kPosition);
        }
        else {
            arm_hold = 0;
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
    else if(r_operator.GetAButton()) {
        extension_state = 2;
    }
    else if(r_operator.GetRightY() != 0) {
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