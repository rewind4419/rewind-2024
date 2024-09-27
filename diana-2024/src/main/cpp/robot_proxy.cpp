// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "robot_proxy.h"
#include "robot.h"

static Robot robot;

void RobotProxy::RobotInit() 
{
	robotInit(&robot);
}

void RobotProxy::RobotPeriodic()
{

}

void RobotProxy::AutonomousInit() 
{
	robotModeChange(&robot, BOTMODE_AUTO);
}

void RobotProxy::AutonomousPeriodic()
{
	robotUpdate(&robot, BOTMODE_AUTO);
}

void RobotProxy::TeleopInit() 
{
	robotModeChange(&robot, BOTMODE_TELEOP);
}

void RobotProxy::TeleopPeriodic() 
{
	robotUpdate(&robot, BOTMODE_TELEOP);
}

void RobotProxy::DisabledInit()
{
	robotModeChange(&robot, BOTMODE_DISABLED);
}

void RobotProxy::DisabledPeriodic() 
{
	robotUpdate(&robot, BOTMODE_DISABLED);
}

void RobotProxy::TestInit() {}
void RobotProxy::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<RobotProxy>();
}
#endif
