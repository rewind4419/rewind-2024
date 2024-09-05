// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_led.SetLength(kLength);
  m_led.SetData(m_ledBuffer);
  m_led.Start();
  
  
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  
  bool ballSensor {!(ballSensorInverted.Get())};
  if (ballSensor == true)
    {
      std::cout << "true";
    // frc::SmartDashboard::PutBoolean("LED ON", ballSensor);
    // COLOR HSV values
    // SetHSV (color 0-180, saturation/ 0-255, brightness 0-255)
    /*
    m_ledBuffer[2].SetHSV(120,255,255); // red
    m_ledBuffer[4].SetHSV(125,255,255); // yellow
    m_ledBuffer[6].SetHSV(130,255,255); // green
    m_ledBuffer[8].SetHSV(135,255,255); // light blue
    m_ledBuffer[10].SetHSV(140,255,255); //med blue
    m_ledBuffer[12].SetHSV(145,255,255); // royal blue
    m_ledBuffer[14].SetHSV(150,255,255); //purple
    m_ledBuffer[15].SetHSV(155,255,255); // Team Purple
    m_ledBuffer[16].SetHSV(160,255,255); //magenta
    m_ledBuffer[17].SetHSV(165,255,255); //magenta
    m_ledBuffer[18].SetHSV(170,255,255); // red
    m_ledBuffer[19].SetHSV(180,255,255); // red 
    */


    /*
    m_ledBuffer[2].SetHSV(120,255,255); //
    m_ledBuffer[4].SetHSV(125,255,255); //
    m_ledBuffer[6].SetHSV(130,255,255); // 
    m_ledBuffer[8].SetHSV(135,255,255); // team purple
    m_ledBuffer[10].SetHSV(140,255,255); //
    m_ledBuffer[12].SetHSV(145,255,255); //
    m_ledBuffer[14].SetHSV(150,255,255); //
    m_ledBuffer[15].SetHSV(155,255,255); //
    m_ledBuffer[16].SetHSV(160,255,255); //
    m_ledBuffer[17].SetHSV(165,255,255); // 
    m_ledBuffer[18].SetHSV(170,255,255); // team magenta
    m_ledBuffer[19].SetHSV(180,255,255); // red
    */


    for (int i = 0; i < kLength; i++) {
      m_ledBuffer[i].SetHSV(145, 255, 255);
      }
    }
  else
    {
      for (int i = 0; i < kLength; i++) {
      m_ledBuffer[i].SetHSV(170, 255, 10);
    }

      m_led.SetData(m_ledBuffer);
    }

  m_led.SetData(m_ledBuffer);
  frc::SmartDashboard::PutBoolean(" BallSensor Detect", ballSensor);

}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
