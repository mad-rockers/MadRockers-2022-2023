// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>

using namespace std;

void Robot::RobotInit() {
  r_driver.SetSquareScale(true);

  r_left_back.Follow(r_left_front);
  r_right_back.Follow(r_right_front);
  r_left_front.SetInverted(true);
  r_right_front.SetInverted(false);
  r_extension.SetInverted(true);

  r_left_pid.SetP(0.1);
  r_left_pid.SetI(0);
  r_left_pid.SetD(0);
  r_left_pid.SetFF(0);

  r_right_pid.SetP(0.1);
  r_right_pid.SetI(0);
  r_right_pid.SetD(0);
  r_right_pid.SetFF(0);

  r_arm_pid.SetP(0.07); /* I believe that lowering this value will slow down the movement of the arm */
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

  r_auto_mode.AddOption("Charge Station", "Charge Station");
  r_auto_mode.SetDefaultOption("No Charge Station", "No Charge Station");
  SmartDashboard::PutData("Auto Mode", &r_auto_mode);
}

void Robot::RobotPeriodic() {
  SmartDashboard::PutNumber("Left", r_left_encoder.GetPosition());
  SmartDashboard::PutNumber("Right", r_right_encoder.GetPosition());
  SmartDashboard::PutNumber("Arm", r_arm_encoder.GetPosition());
  SmartDashboard::PutNumber("Extension", r_extension_encoder.GetPosition());
  SmartDashboard::PutNumber("Gyro Angle", double(r_gyro.GetAngle()));
  SmartDashboard::PutNumber("Gyro Rate", double(r_gyro.GetRate()));
  //SmartDashboard::PutNumber("Gyro Rate", double(arm_limit_high.Get()));
}

//This code only runs once at the beginning of autonomous
void Robot::AutonomousInit() {
  //Begin by resetting and rezeroing variables and encoders
  stop_all();
  auto_state = 0; //This variable here controls what step in the auto code to run. It starts at 0.
  left_hold = 0;
  r_left_encoder.SetPosition(0);
  r_right_encoder.SetPosition(0);
  r_gyro.Reset();
  timer1.Stop();
  timer1.Reset();
}

//This code runs in an infinite loop all throughout autonomous
void Robot::AutonomousPeriodic() {
  /*
  This section is just dedicated to initializing some variables for how the auto code will run.
  The benefit of putting the settings up here is that they are easy and fast to tweak.
  */
  double arm_place_high;
  double arm_place_low;
  double extension_place;
  int tolerance;
  
  
  arm_place_high= -75;  //Defines the high position for the arm
  arm_place_low = -25;  //Defines the low position for the arm
  extension_place = -280;  //Defines the stretch out position for the extension
  tolerance = 2;  // This is used to set tolerance of how close the encoder needs to be to our defined location


  /*
  This switch statement is where the actual auto runs.
  Depending on what auto_state is, it will only run 1 of the steps.
  Each step is under a "case" statement.
  So only one of the sections under a case statement will be running at a time.

  Step descriptions:
  0 - Move extension to 0 position (I think we can remove)
  1 - Raise the arm
  2 - Lower the arm halfway
  */
  switch(auto_state) {
    case 0:
      /*
      Sets the extension to 0 position
      I don't think we'll need this for parade wave
      */
      r_extension_pid.SetReference(0, ControlType::kPosition);

      auto_state++;
      
      break;
    
    case 1:

      //Move the arm to the high position
      r_arm_pid.SetReference(arm_place_high, ControlType::kPosition);

      /*
      This is a check to see when the motor gets to the defined position. We give it a static number for a buffer.
      When it gets there we move to next state.

      If I need to get the position that a motor is currently in, I can do so by using its built-in encoder.
      The GetPosition() command returns how many rotations the MOTOR has rotated from the 0 point.
      
      */
      if(abs(r_arm_encoder.GetPosition() - arm_place_high) < tolerance) {
        // arm_place = 0;
        auto_state++;
        
      }

      break;

    case 2:
      //sleep for 0.5 seconds
      if(timer1.Get() == 0_s) {
        timer1.Start();
      }
      //if (timer1.Get() > 1.0_s) {
      if(timer1.Get() > 0.5_s) {
        timer1.Stop();
        timer1.Reset();
        auto_state++;
      }
      break;

    case 3:
      // This is moving to the low arm position
      r_arm_pid.SetReference(arm_place_low, ControlType::kPosition);

      /*
      Same as in case 1, we move use encoder to check if it is with a tolerance level close to desired position
      If I need to get the position that a motor is currently in, I can do so by using its built-in encoder.
      The GetPosition() command returns how many rotations the MOTOR has rotated from the 0 point.*/
      if(abs(r_arm_encoder.GetPosition() - arm_place_low) < tolerance) {
        auto_state++;
        break;
      }
      break;

      case 4:
      //sleep for 0.5 seconds
      if(timer1.Get() == 0_s) {
        timer1.Start();
      }
      //if (timer1.Get() > 1.0_s) {
      if(timer1.Get() > 0.5_s) {
        timer1.Stop();
        timer1.Reset();
        auto_state = 1;
      }
      break;

    


  }
}

void Robot::TeleopInit() {
  stop_all();
  arm_state = 0;
  extension_state = 0;
  arm_hold = 0;
  extension_hold = 0;
  r_left_front.SetInverted(true);
  r_right_front.SetInverted(false);
}

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
  stop_all();
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
  r_arm_pid.SetReference(-2, ControlType::kPosition);
  r_extension_pid.SetReference(-45, ControlType::kPosition);
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
