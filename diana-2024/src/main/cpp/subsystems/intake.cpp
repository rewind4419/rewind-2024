#include "intake.h"

#include "../maths.h"

#include "../config.h"

void initIntake(Robot* robot)
{
	initPid(&robot->intake.pid);
	robot->intake.target_angle = 0.2;
}

void updateCubeIntake(Robot* robot)
{

	float encoder_val = 0;
	encoder_val = robot->intake.intake_encoder.GetAbsolutePosition();

	// std::cout << encoder_val << std::endl;
	// Starting angle is 0 then pi/2 is vertical pi is down
	robot->intake.currentAngle = robot->intake.intake_encoder.GetAbsolutePosition() - INTAKE_AXIS_OFFSET;

	float delta_angle = robot->intake.target_angle - robot->intake.currentAngle;

	float curr_intake_pid = evalPid(&robot->intake.pid, delta_angle, robot->delta_time);

	float target_throttle = CLAMP(curr_intake_pid, -0.8, 0.8);

	robot->intake.actual_throttle = target_throttle - 0.025 * cos((robot->intake.intake_encoder.GetAbsolutePosition() - .25)* M_PI);

	// robot->intake.actual_throttle = mix(
	// 	robot->intake.actual_throttle, 
	// 	target_throttle, 
	// 	0.2f
	// );

	robot->intake.axis_motor.Set(robot->intake.actual_throttle);
	robot->intake.intake_motor.Set(robot->intake.puller_speed);
	//printf("test throttle: %f \n", robot->intake.actual_throttle);
}