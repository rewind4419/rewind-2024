#pragma once

#include "hw_map.h"
#include "gamepad.h"
#include "fennec/pid.h"
#include <frc/DutyCycleEncoder.h>


enum RobotMode
{
    BOTMODE_TELEOP,
    BOTMODE_AUTO,
    BOTMODE_DISABLED,
};

struct Drivetrain
{
	float throttle = 0;
	float steer = 0;
};


struct Robot
{

	RobotHardware hw;

	Drivetrain drive;
	Input input;

	float delta_time = 0.02;
	float auto_timer = 0;

};

void robotInit(Robot* robot);

void robotModeChange(Robot* robot, RobotMode mode);
void robotUpdate(Robot* robot, RobotMode mode);
