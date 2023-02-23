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
  drive();
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
