#include "Robot.h"

void Robot::drive() {
    float speed;
    if(driver_right.GetTrigger()) {
        speed = 0.5;
    }
    else {
        speed = 1;
    }

    float left_power, right_power;

    if(left_front.GetInverted()) {
        left_power = driver_right.GetY() * speed;
        right_power = driver_left.GetY() * speed;
    }
    else {
        left_power = driver_left.GetY() * speed;
        right_power = driver_right.GetY() * speed;
    }
    
    drivetrain.TankDrive(left_power, right_power, false);

    if(driver_left.GetTriggerPressed()) {
        left_front.SetInverted(!left_front.GetInverted());
        right_front.SetInverted(!right_front.GetInverted());
    }
}