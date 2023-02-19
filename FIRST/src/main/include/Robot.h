// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/drive/DifferentialDrive.h>
#include <frc/DigitalInput.h>
#include <frc/ADIS16470_IMU.h>
#include "rev/CANSparkMax.h"
#include "rev/ColorSensorV3.h"

#include "Ports.h"
#include "CustomController.h"

using namespace frc;
using namespace rev;

class Robot : public frc::TimedRobot {
 public:
  CustomController driver;
  CANSparkMax left_front;
  CANSparkMax left_back;
  CANSparkMax right_front;
  CANSparkMax right_back;
  /*CANSparkMax arm;
  SparkMaxPIDController arm_pid = arm.GetPIDController();
  CANSparkMax extension;
  SparkMaxPIDController extension_pid = extension.GetPIDController();*/
  DifferentialDrive drivetrain;
  ADIS16470_IMU gyro;
  SendableChooser<ADIS16470_IMU::IMUAxis> axis_chooser;
  /*DigitalInput arm_limit_low;
  DigitalInput arm_limit_high;
  DigitalInput extension_limit_back;
  DigitalInput extension_limit_front;
  ColorSensorV3 color_sensor;*/

  Robot() :
  driver(Ports::driver),
  left_front(Ports::left_front, CANSparkMax::MotorType::kBrushless),
  left_back(Ports::left_back, CANSparkMax::MotorType::kBrushless),
  right_front(Ports::right_front, CANSparkMax::MotorType::kBrushless),
  right_back(Ports::right_back, CANSparkMax::MotorType::kBrushless),
  /*arm(Ports::arm, CANSparkMax::MotorType::kBrushless),
  extension(Ports::extension, CANSparkMax::MotorType::kBrushless),*/
  drivetrain(left_front, right_front)
  /*arm_limit_low(Ports::arm_limit_low),
  arm_limit_high(Ports::arm_limit_high),
  extension_limit_back(Ports::extension_limit_back),
  extension_limit_front(Ports::extension_limit_front),
  color_sensor(Ports::color_sensor)*/
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
  double GetLimelightValue(std::string);
  std::vector<double> GetLimelightArray(std::string);
  void SetLimelightValue(std::string, double);

  private:
};
