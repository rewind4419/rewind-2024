#include "robot.h"

#include "math.h"
#include "maths.h"
#include <iostream>

// Subsystems
#include "subsystems/drivetrain.h"

#include "frc/Joystick.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class TestCommand
    : public frc2::CommandHelper<frc2::Command, TestCommand>
{
	TestCommand();

	void Initialize();
	void Execute();
	bool IsFinished();

	int counter = 0;
};

void TestCommand::Initialize()
{
	printf("Init command\n");
}

void TestCommand::Execute()
{
	printf("Execute order 66\n");
	counter++;
}

bool TestCommand::IsFinished()
{
	printf("Finish check\n");

	if (this->counter < 3)
	{
		printf("^Returning true\n");
		return true;
	}

	return false;
}

static frc::Joystick gamepad(0);


void robotInit(Robot* robot)
{
	hwInit(&robot->hw);
	//initIntake(robot);
	frc::SmartDashboard::GetNumber("Auton Seconds", 1.4);
	frc::SmartDashboard::GetNumber("Auton Speed", 0.4);
}

float auto_timer = 0;
void robotModeChange(Robot* robot, RobotMode mode)
{
	auto_timer = 0;

	if (mode == BOTMODE_TELEOP)
	{
	}
}

void robotUpdate(Robot* robot, RobotMode mode)
{
	if (mode == BOTMODE_TELEOP)
	{
		updateGamepad(&robot->input);

		updateDrivetrain(robot, robot->input.driver.joystick_left.x, robot->input.driver.trigger_right - robot->input.driver.trigger_left);
	}

	if (mode == BOTMODE_AUTO)
	{
		// // DISABLED

		// const float AUTO_OUTTAKE_CHECKPOINT = 3;
		// const float AUTO_TIME_END = 6;
		// robot->auto_timer += robot->delta_time;

		// if (robot->auto_timer < AUTO_OUTTAKE_CHECKPOINT)
		// {
		// 	robot->intake.target_angle = 0.75;
		// 	robot->intake.puller_speed = -1;
		// }
		// else if (robot->auto_timer < AUTO_TIME_END && robot->auto_timer > AUTO_OUTTAKE_CHECKPOINT)
		// {
		// 	updateDrivetrain(robot, .264, -0.4f);
		// 	robot->intake.target_angle = 0.25;
		// 	robot->intake.puller_speed = 0;
		// }
		// else
		// {
		// 	robot->intake.target_angle = 0.25;
		// 	updateDrivetrain(robot, 0, 0);
		// }
		// updateCubeIntake(robot);
	}
}
