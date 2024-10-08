// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include "RobotContainer.h"

#include <frc/DigitalInput.h>

#include <frc/AddressableLED.h>
#include <array>
#include <math.h>


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

 private:
  // Have it empty by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  std::optional<frc2::CommandPtr> m_autonomousCommand;
  //PhotoElecSensor
   frc::DigitalInput m_ballSensorInverted{0};
  // LED
  // assign length of LEDs
  static constexpr int m_kLength{20};
  //PWM port 9
  frc::AddressableLED m_led{9};
  //set m_ledBuffer to these things
  std::array<frc::AddressableLED::LEDData, m_kLength> m_ledBuffer; //reuse the buffer
  RobotContainer m_container;
};
