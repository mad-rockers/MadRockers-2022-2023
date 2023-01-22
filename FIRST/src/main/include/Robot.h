// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include "rev/CANSparkMax.h"

#include "Ports.h"
#include "CustomController.h"

using namespace frc;
using namespace rev;

class Robot : public frc::TimedRobot {
 public:

  CustomController driver;
  VictorSP left_front;
  VictorSP left_back;
  VictorSP right_front;
  VictorSP right_back;
  MotorControllerGroup left_drive;
  MotorControllerGroup right_drive;
  /*CANSparkMax left_front;
  CANSparkMax left_back;
  CANSparkMax right_front;
  CANSparkMax right_back;*/
  DifferentialDrive drivetrain;

  Robot() :
  driver(Ports::driver),
  left_front(Ports::left_front),
  left_back(Ports::left_back),
  right_front(Ports::right_front),
  right_back(Ports::right_back),
  left_drive(left_front, left_back),
  right_drive(right_front, right_back),
  /*left_front(Ports::left_f, CANSparkMax::MotorType::kBrushless),
  left_back(Ports::left_b, CANSparkMax::MotorType::kBrushless),
  right_front(Ports::right_f, CANSparkMax::MotorType::kBrushless),
  right_back(Ports::right_b, CANSparkMax::MotorType::kBrushless),*/
  drivetrain(left_drive, right_drive)
  {}

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
  
  //Custom functions
  double limelight_get(std::string, double);
  void limelight_set(std::string, double);
};
