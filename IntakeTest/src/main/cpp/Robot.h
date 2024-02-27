// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include "fennec/maths.h"
#include "fennec/gamepad.h"
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

class Robot : public frc::TimedRobot {
 public:
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

struct RobotData
{
  // high level
	// TaskMgr taskmgr;

  // control
	float delta_time;
  float enable_time;
  float imu_basis;

  float auto_imu_basis;

  bool middle_wheels = false;
 // Aligner aligner = ALGN_NONE;

  //PID aligner_pid { .kP = 1.5, .kI = 0, .kD = 0.2 };
  //PID holder_pid { .kP = 1.5, .kI = 0, .kD = 0.2 };

  Input input;
  //Localiser_FirstOrderLag localiser;
  //OdometryFrame latest_odometry_frame; 

 // DrivetrainController drivetrain_controller;

//   ChargingPad charging_pad;


  // physical components
  //Drivetrain drivetrain;
  //AHRS* sensor_imu;

  float driver_speed;

  float held_rotation;

  //v3 integrated_imu_pos;

  // misc
  //frc::Field2d field;

  

  // assemblies
//   Hank hank;
//   Stag stag;
//   Shrek shrek;
//   Intake intake;
//   Led led;

  //
//   MateSystem mate_system;
}; 