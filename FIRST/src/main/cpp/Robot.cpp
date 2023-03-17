// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>

void Robot::RobotInit() {
  r_left_back.Follow(r_left_front);
  r_right_back.Follow(r_right_front);
  r_left_front.SetInverted(true);
  r_right_front.SetInverted(false);

  r_compressor.EnableAnalog(60_psi, 120_psi);

  CameraServer::StartAutomaticCapture();
  CameraServer::StartAutomaticCapture();

  axis_chooser.AddOption("X", ADIS16470_IMU::IMUAxis::kX);
  axis_chooser.AddOption("Y", ADIS16470_IMU::IMUAxis::kY);
  axis_chooser.SetDefaultOption("Z", ADIS16470_IMU::IMUAxis::kZ);
  SmartDashboard::PutData("Gyro Axis", &axis_chooser);
}

void Robot::RobotPeriodic() {
  if(r_gyro.GetYawAxis() != axis_chooser.GetSelected()) {
    r_gyro.SetYawAxis(axis_chooser.GetSelected());
  }
  SmartDashboard::PutNumber("Gyro Angle", double(r_gyro.GetAngle()));
  SmartDashboard::PutNumber("Gyro Rate", double(r_gyro.GetRate()));

  SmartDashboard::PutNumber("Arm Limit Low", r_arm_limit_low.Get());
  SmartDashboard::PutNumber("Arm Limit High", r_arm_limit_high.Get());
  SmartDashboard::PutNumber("Extension Limit Back", r_extension_limit_back.Get());
  SmartDashboard::PutNumber("Extension Limit Front", r_extension_limit_front.Get());
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  drivetrain();
  arm();
  extension();
  box();
  grabber();
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
