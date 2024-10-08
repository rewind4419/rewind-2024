// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor.RestoreFactoryDefaults();
    
    // set PID coefficients
    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
    m_pidController.SetIZone(kIz);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);  
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
  bool ballSensor {!(m_ballSensorInverted.Get())};
  frc::SmartDashboard::PutBoolean("  BallSensor", ballSensor);

  // read PID coefficients from SmartDashboard
  double p = frc::SmartDashboard::GetNumber("  P Gain", 0);
  double i = frc::SmartDashboard::GetNumber("  I Gain", 0);
  double d = frc::SmartDashboard::GetNumber("  D Gain", 0);
  double iz = frc::SmartDashboard::GetNumber("  I Zone", 0);
  double ff = frc::SmartDashboard::GetNumber("  Feed Forward", 0);
  double max = frc::SmartDashboard::GetNumber("  Max Output", 0);
  double min = frc::SmartDashboard::GetNumber("  Min Output", 0);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if((p != kP)) { m_pidController.SetP(p); kP = p; }
  if((i != kI)) { m_pidController.SetI(i); kI = i; }
  if((d != kD)) { m_pidController.SetD(d); kD = d; }
  if((iz != kIz)) { m_pidController.SetIZone(iz); kIz = iz; }
  if((ff != kFF)) { m_pidController.SetFF(ff); kFF = ff; }
  if((max != kMaxOutput) || (min != kMinOutput)) { 
    m_pidController.SetOutputRange(min, max); 
    kMinOutput = min; kMaxOutput = max; 
  }

  // read setpoint from joystick and scale by max rpm
  double SetPoint = 0.0;// = MaxRPM*m_stick.GetY();

  /* comment out joystick code since using photosensor
  if (m_stick.GetRawButton(1)) {
    SetPoint = 100;
  } else if (m_stick.GetRawButton(2)) {
    SetPoint = 500;
  } else if (m_stick.GetRawButton(3)) {
    SetPoint = 750;
  } else if (m_stick.GetRawButton(4)) {
    SetPoint = 2500;
  } else {
    SetPoint = 0;
  }
  */

  if (ballSensor == true) {
      SetPoint=100;
    } else {
      SetPoint = 0;
    }

  /**
   * PIDController objects are commanded to a set point using the 
   * SetReference() method.
   * 
   * The first parameter is the value of the set point, whose units vary
   * depending on the control type set in the second parameter.
   * 
   * The second parameter is the control type can be set to one of four 
   * parameters:
   *  rev::CANSparkMax::ControlType::kDutyCycle
   *  rev::CANSparkMax::ControlType::kPosition
   *  rev::CANSparkMax::ControlType::kVelocity
   *  rev::CANSparkMax::ControlType::kVoltage
   */
    
    m_pidController.SetReference(SetPoint, rev::CANSparkFlex::ControlType::kVelocity);
    m_motor.Set(SetPoint);

    frc::SmartDashboard::PutNumber("  SetPoint", SetPoint);
    frc::SmartDashboard::PutNumber("  ProcessVariable", m_encoder.GetVelocity());
  
  
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
