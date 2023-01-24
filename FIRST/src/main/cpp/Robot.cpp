// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  /*left_back.Follow(left_front);
  right_back.Follow(right_front);

  left_front.SetInverted(false);
  right_front.SetInverted(false);*/
  left_front.SetInverted(true);
  left_drive.SetInverted(false);
  right_drive.SetInverted(false);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  const float brake = 0.01;

  float speed;
  if(driver.GetRightBumper()) {
    speed = 0.5;
  }
  else {
    speed = 1;
  }

  float left_power = driver.GetLeftY() * speed + brake;
  float right_power = driver.GetRightY() * speed + brake;
  drivetrain.TankDrive(left_power, right_power, false);

  if(driver.GetAButton()) {
    while(driver.GetAButton()) {}
    /*left_front.SetInverted(!left_drive.GetInverted());
    right_front.SetInverted(!right_drive.GetInverted());*/
    left_drive.SetInverted(!left_drive.GetInverted());
    right_drive.SetInverted(!right_drive.GetInverted());
  }

  box.Set(driver.GetLeftTriggerAxis() - driver.GetRightTriggerAxis());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
