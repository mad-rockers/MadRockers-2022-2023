// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  axis_chooser.AddOption("X", ADIS16470_IMU::IMUAxis::kX);
  axis_chooser.AddOption("Y", ADIS16470_IMU::IMUAxis::kY);
  axis_chooser.SetDefaultOption("Z", ADIS16470_IMU::IMUAxis::kZ);
  SmartDashboard::PutData("Gyro Axis", &axis_chooser);

  left_back.Follow(left_front);
  right_back.Follow(right_front);
  left_front.SetInverted(false);
  right_front.SetInverted(false);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  if(gyro.GetYawAxis() != axis_chooser.GetSelected()) {
    gyro.SetYawAxis(axis_chooser.GetSelected());
  }
  SmartDashboard::PutNumber("Gyro Angle", double(gyro.GetAngle()));
  SmartDashboard::PutNumber("Gyro Rate", double(gyro.GetRate()));
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
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
