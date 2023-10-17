#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

/* 
This function stops all robot components, 
resets timers, and sets various motors and controls to their default values.
*/
void Robot::stop_all() {
    r_left_front.Set(0);
    r_right_front.Set(0);
    r_arm.Set(0);
    r_extension.Set(0);
    r_box.Set(false);
    timer1.Stop();
    timer1.Reset();
    timer2.Stop();
    timer2.Reset();
}

/* 
This function controls the robot's drivetrain, including velocity control, acceleration, 
and deceleration.
*/
void Robot::drivetrain() {
    const float maxRPM = 5000;
    float slow_speed = 0.3;
    float max_speed = 1;
    units::second_t accel_time = 0.75_s;
    units::second_t deccel_time = 0.75_s;
    float deccel_trigger = 1700;

    float speed;

    if(r_operator.GetLeftBumper()) {
        if(timer1.Get() == 0_s) {
            timer1.Start();
        }
        if(timer1.Get() < accel_time) {
            speed = double(timer1.Get()) / double(accel_time) * (max_speed - slow_speed) + slow_speed;
        }
        else {
            speed = max_speed;
        }
    }
    else {
        timer1.Stop();
        timer1.Reset();
        speed = slow_speed;
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

    if(left_power == 0 && right_power == 0) {
        if(abs(r_left_encoder.GetVelocity()) > deccel_trigger || abs(r_right_encoder.GetVelocity()) > deccel_trigger) {
            timer2.Start();
        }
    }
    else {
        timer2.Stop();
        timer2.Reset();
    }
    if(timer2.Get() > 0_s && timer2.Get() < deccel_time) {
        float l_setting = r_left_encoder.GetVelocity() / maxRPM;
        float r_setting = r_right_encoder.GetVelocity() / maxRPM;
        left_power = (float(deccel_time) - float(timer2.Get())) / float(deccel_time) * l_setting;
        right_power = (float(deccel_time) - float(timer2.Get())) / float(deccel_time) * r_setting;
    }


    if(left_power == 0 && right_power == 0 && r_operator.GetRightTriggerAxis()) {
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
    

    /*if(r_driver.GetAButton()) {
        while(r_driver.GetAButton()) {}
        r_left_front.SetInverted(!r_left_front.GetInverted());
        r_right_front.SetInverted(!r_right_front.GetInverted());
    }*/
}

/*
This function controls the robot's arm mechanism, allowing for various states, 
such as normal control, zero position, and grabbing position.

Triangle button moves arm to 0 position
Cross button moves arm to grabbing position
When left stick is moved vertically (y axis) sets arm to normal control

*/
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

    if(arm_state == 1 && r_extension_encoder.GetPosition() > -10) {
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

/*
This function controls the robot's extension mechanism, including different states 
like zero position and grabbing position.

Square button moves extension to 0 position
When right stick is moved vertically (y axis) sets extension to normal control
Circle button moved extention to grabbin position
*/
void Robot::extension() {
    /*
    Extension States:
    0 - Normal control
    1 - Zero position
    2 - Grabbing position
    */
    double grab = -46;

    if(r_operator.GetXButton()) {
        extension_state = 1;
    }
    else if(r_operator.GetBButton()) {
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
        double setpoint = r_driver.GetRightY();
        if(r_extension_encoder.GetPosition() < -274 && setpoint < 0) {
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

/*
This function controls a box mechanism used to drop off items

R1 moves the dumping box
*/
void Robot::box() {
    if(r_driver.GetR1Button()) {
        r_box.Set(true);
    }
    else {
        r_box.Set(false);
    }
}


/*This function handles a grabber mechanism and opens or closes it based on user input.

Left bumper opens the grabber
Right bumper closes the grabber low
Right trigger axis 
*/
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

// This function opens a grabber mechanism.
void Robot::grabber_open() {
    r_low_grabber.Set(true);
    r_high_grabber.Set(false);
}

// This function closes a grabber mechanism in a low position.
void Robot::grabber_close_low() {
    r_low_grabber.Set(false);
    r_high_grabber.Set(false);
}
// This function closes a grabber mechanism in a high position.
void Robot::grabber_close_high() {
    r_low_grabber.Set(false);
    r_high_grabber.Set(true);
}