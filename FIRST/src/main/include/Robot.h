// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/VictorSP.h>
#include "rev/CANSparkMax.h"

#include "Ports.h"
#include "CustomController.h"

using namespace frc;
using namespace rev;

class Robot : public frc::TimedRobot {
 public:

  CustomController driver;
  DifferentialDrive drivetrain;
  CANSparkMax left_f;
  CANSparkMax left_b;
  CANSparkMax right_f;
  CANSparkMax right_b;

  Robot() :
  driver(Ports::driver),
  drivetrain(left_f, right_f),
  left_f(Ports::left_f, CANSparkMax::MotorType::kBrushless),
  left_b(Ports::left_b, CANSparkMax::MotorType::kBrushless),
  right_f(Ports::right_f, CANSparkMax::MotorType::kBrushless),
  right_b(Ports::right_b, CANSparkMax::MotorType::kBrushless)
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

 private:
  //Custom functions
  double limelight_get(std::string, double);
  void limelight_set(std::string, double);
};
