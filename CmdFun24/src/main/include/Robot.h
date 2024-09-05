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
   frc::DigitalInput ballSensorInverted{0};


  // LED
  // assign length of LEDs
  static constexpr int kLength = 20;
  //PWM port 9
  frc::AddressableLED m_led{9};
  //set m_ledBuffer to these things
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer; //reuse the buffer


  // Our LED strip has density of 120 LED per meter
  // units::meter_t kLedSpacing{1 / 120.0};

  // Create LED pattern raibow
  // all hues at max saturation and half brightness
  // frc::LEDPattern m_rainbow = frc::LEDPattern::Rainbow(255,128);


  // Create new pattern that scrolls rainbow
  // frc::LEDPattern m_scrollingRainbow = mrainbow.ScrollAtAbsoluteSpeed(1_mps, kLedSpacing);

  RobotContainer m_container;
};
