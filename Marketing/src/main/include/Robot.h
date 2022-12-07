// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/drive/MecanumDrive.h>
#include "CustomController.h"
#include "Ports.h"

using namespace frc;

class Robot : public frc::TimedRobot {
 public:
  CustomController controller;
  VictorSP front_l;
  VictorSP front_r;
  VictorSP back_l;
  VictorSP back_r;
  MecanumDrive drivetrain;
  
  Robot() :
  controller(ports::controller),
  front_l(ports::front_l),
  front_r(ports::front_r),
  back_l(ports::back_l),
  back_r(ports::back_r),
  drivetrain(front_l, back_l, front_r, back_r)
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
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
