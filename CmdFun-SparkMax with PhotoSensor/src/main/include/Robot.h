// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
// #include <optional>
#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include "RobotContainer.h"
#include <frc/DigitalInput.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkBase.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkLowLevel.h>

// #include <frc/AddressableLED.h>
// #include <array>
// #include <math.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  // initialize motor
  static const int deviceID = 9;

  // sparkflex
  rev::CANSparkFlex m_motor{deviceID, rev::CANSparkLowLevel::MotorType::kBrushless};
  //sparkmax
  // rev::CANSparkMax m_motor{deviceID, rev::CANSparkMax::MotorType::kBrushless};


  /**
   * In order to use PID functionality for a controller, a SparkPIDController object
   * is constructed by calling the GetPIDController() method on an existing
   * CANSparkMax object
   */
  rev::SparkPIDController m_pidController = m_motor.GetPIDController();

  // Encoder object created to display velocity values
  // Neo and 550 =  42 counts per rev
  // Sparkflex =  7168 counts per rev
  rev::SparkRelativeEncoder m_encoder = m_motor.GetEncoder();

  //comment out joystick and use the photoelec sensor
  //frc::Joystick m_stick{0};

  // default PID coefficients
  double kP = 6e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0;

  // motor max RPM
  const double MaxRPM = 5000;


 private:
  // Have it empty by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  std::optional<frc2::CommandPtr> m_autonomousCommand;
  //PhotoElecSensor
   frc::DigitalInput m_ballSensorInverted{0};


  RobotContainer m_container;
};
