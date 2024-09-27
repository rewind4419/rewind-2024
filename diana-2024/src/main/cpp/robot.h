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

struct Intake
{
	float currentAngle;
	float target_angle = 0;
	float puller_speed = 0.2;

	bool extended = false;
	bool outtaking = false;


	PID pid = { .kP = 1.1f, .kI = 0, .kD = 0,  .errorAccum = 0, .lastError = 0 };
	float actual_throttle = 0;

	rev::CANSparkMax axis_motor     { 6, rev::CANSparkMax::MotorType::kBrushless };
    rev::CANSparkMax intake_motor     { 7, rev::CANSparkMax::MotorType::kBrushless };

	frc::DutyCycleEncoder intake_encoder { 0 };

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
