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
  r_extension.SetInverted(true);

  r_left_pid.SetP(0.08);
  r_left_pid.SetI(0);
  r_left_pid.SetD(0);
  r_left_pid.SetFF(0);

  r_right_pid.SetP(0.08);
  r_right_pid.SetI(0);
  r_right_pid.SetD(0);
  r_right_pid.SetFF(0);

  r_arm_pid.SetP(0.07);
  r_arm_pid.SetI(0);
  r_arm_pid.SetD(0);
  r_arm_pid.SetFF(0);

  r_extension_pid.SetP(0.05);
  r_extension_pid.SetI(0);
  r_extension_pid.SetD(0);
  r_extension_pid.SetFF(0);

  r_compressor.EnableAnalog(60_psi, 120_psi);

  r_gyro.SetYawAxis(ADIS16470_IMU::IMUAxis::kY);

  CameraServer::StartAutomaticCapture();
  CameraServer::StartAutomaticCapture();

  r_auto_mode.SetDefaultOption("Charge Station", "Charge Station");
  r_auto_mode.AddOption("No Charge Station", "No Charge Station");
  SmartDashboard::PutData("Auto Mode", &r_auto_mode);
}

void Robot::RobotPeriodic() {
  SmartDashboard::PutNumber("Gyro Angle", double(r_gyro.GetAngle()));
  SmartDashboard::PutNumber("Gyro Rate", double(r_gyro.GetRate()));

  SmartDashboard::PutNumber("Left", r_left_encoder.GetPosition());
  SmartDashboard::PutNumber("Right", r_right_encoder.GetPosition());
  SmartDashboard::PutNumber("Arm", r_arm_encoder.GetPosition());
  SmartDashboard::PutNumber("Extension", r_extension_encoder.GetPosition());
}

void Robot::AutonomousInit() {
  auto_state = 0;
  r_left_encoder.SetPosition(0);
  r_right_encoder.SetPosition(0);
  r_gyro.Reset();
  timer.Stop();
  timer.Reset();
}

void Robot::AutonomousPeriodic() {
  double arm_place;
  double extension_place;
  if(r_auto_mode.GetSelected() == "Charge Station") {
    arm_place = -75;
    extension_place = -300;
  }
  else {
    arm_place = -75;
    extension_place = -270;
  }
  double drive_speed = 0.3;
  double initial_back = 70;
  double charge_forward = 35;
  double balance_speed = 0.1;
  units::degree_t balance_zone = 5_deg;
  double balance_rate = 10;
  switch(auto_state) {
    case 0:
      if(r_auto_mode.GetSelected() == "Charge Station") {
        auto_state++;
        break;
      }
      grabber_close_high();
      if(timer.Get() == 0_s) {
        timer.Start();
      }
      if(timer.Get() > 0.5_s) {
        timer.Stop();
        timer.Reset();
        auto_state++;
      }
      break;
    
    case 1:
      r_arm_pid.SetReference(arm_place, ControlType::kPosition);
      if(abs(r_arm_encoder.GetPosition() - arm_place) < 2) {
        auto_state++;
      }
      break;

    case 2:
      r_extension_pid.SetReference(extension_place, ControlType::kPosition);
      if(abs(r_extension_encoder.GetPosition() - extension_place) < 1) {
        auto_state++;
      }
      break;

    case 3:
      grabber_open();
      if(timer.Get() == 0_s) {
        timer.Start();
      }
      if(timer.Get() > 0.5_s) {
        auto_state++;
      }
      break;
    
    case 4:
      r_extension_pid.SetReference(0, ControlType::kPosition);
      r_left_front.Set(drive_speed);
      r_right_front.Set(drive_speed);
      if(r_extension_encoder.GetPosition() > -20) {
        auto_state++;
      }
      break;
    
    case 5:
      r_arm_pid.SetReference(0, ControlType::kPosition);
      if(r_left_encoder.GetPosition() > initial_back) {
        r_left_front.Set(0);
        r_right_front.Set(0);
        if(r_auto_mode.GetSelected() == "Charge Station") {
          auto_state++;
        }
      }
      break;

    case 6:
      r_left_front.Set(-drive_speed);
      r_right_front.Set(-drive_speed);
      if(r_left_encoder.GetPosition() < charge_forward) {
        auto_state++;
      }
      break;

    case 7:
      if(r_gyro.GetAngle() < -balance_zone && abs(double(r_gyro.GetRate())) < balance_rate) {
        r_left_front.Set(balance_speed);
        r_right_front.Set(balance_speed);
        left_hold = 0;
      }
      else if(r_gyro.GetAngle() > balance_zone && abs(double(r_gyro.GetRate())) < balance_rate) {
        r_left_front.Set(-balance_speed);
        r_right_front.Set(-balance_speed);
        left_hold = 0;
      }
      else {
        if(left_hold == 0) {
          left_hold = r_left_encoder.GetPosition();
          right_hold = r_right_encoder.GetPosition();
        }
        r_left_pid.SetReference(left_hold, ControlType::kPosition);
        r_right_pid.SetReference(right_hold, ControlType::kPosition);
      }
      break;
  }
}

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

void Robot::TestInit() {
  while(r_extension_limit_back.Get() == 0) {
    r_extension.Set(0.2);
  }
  r_extension_encoder.SetPosition(0);
  while(r_extension_encoder.GetPosition() > -10) {
    r_extension.Set(-0.1);
  }
  while(r_extension_limit_back.Get() == 0) {
    r_extension.Set(0.05);
  }
  r_extension.Set(0);
  r_extension_encoder.SetPosition(0);
  while(r_arm_limit_low.Get() == 0) {
    r_arm.Set(0.2);
  }
  r_arm_encoder.SetPosition(0);
  while(r_arm_encoder.GetPosition() > -5) {
    r_arm.Set(-0.1);
  }
  while(r_arm_limit_low.Get() == 0) {
    r_arm.Set(0.02);
  }
  r_arm.Set(0);
  r_arm_encoder.SetPosition(0);
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
