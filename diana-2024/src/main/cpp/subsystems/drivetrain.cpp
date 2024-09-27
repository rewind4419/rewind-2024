#include "drivetrain.h"

#include "../maths.h"

#include "../config.h"

void updateDrivetrain(Robot* robot, float steer, float throttle)
{
	// printf("Steer %f, Drive %f\n", steer, throttle);

	//raw steering value - dont use
	float raw_steer = steer * fabsf(steer);

	//raw throttle value - dont use
	float raw_throttle = throttle; 

	if (fabsf(raw_steer) < DRIVETRAIN_STEER_DEADZONE) raw_steer = 0;

	if (fabsf(raw_throttle) < DRIVETRAIN_THROTTLE_DEADZONE) raw_throttle = 0;

	raw_steer *= DRIVETRAIN_STEER_MULTIPLIER;

	//process the signal for each motor so it actually works vvv

	//easing throttle value for non suddden acceleration - avoid wheelslip
	float throttle_delta_mul = DRIVETRAIN_THROTTLE_ACCELERATION * delta_time;
	if (throttle_delta_mul > 1.0f) throttle_delta_mul = 1.0f;


	float throttle_delta = (raw_throttle - robot->drive.throttle) * throttle_delta_mul;
	robot->drive.throttle += throttle_delta;


	//easing steering value for non sudden acceleration
	float steer_delta_mul = DRIVETRAIN_STEER_ACCELERATION * delta_time;
	if (steer_delta_mul > 1.0f) steer_delta_mul = 1.0f;

	float steerDelta = (raw_steer - robot->drive.steer) * steer_delta_mul;
	robot->drive.steer += steerDelta;


	//clamp it to avoid horrendously high motor speeds on deploy from 
	robot->drive.steer = CLAMP(robot->drive.steer, -1, 1);
	robot->drive.throttle = CLAMP(robot->drive.throttle, -1, 1);


	//left and right motor throttles
	float motor_left = (robot->drive.steer + robot->drive.throttle) * DRIVETRAIN_THROTTLE_MULTIPLIER;
	float motor_right = (-robot->drive.steer + robot->drive.throttle) * DRIVETRAIN_THROTTLE_MULTIPLIER;


	//clamp the throttles between 1 and -1 to not send absurdly large numbers to the motor drivers
	motor_left = CLAMP(motor_left, -1, 1);
	motor_right = CLAMP(motor_right, -1, 1); 

	//inverted to account for motor side change
	motor_right = -motor_right; 

	// printf("%f %f\n", motor_left, motor_right);
	// push the values to the hardware

	robot->hw.left_motors[0]->Set(robot->hw.controlMode, motor_left);
	robot->hw.left_motors[1]->Set(robot->hw.controlMode, motor_left);

	robot->hw.right_motors[1]->Set(robot->hw.controlMode, motor_right);
	robot->hw.right_motors[0]->Set(robot->hw.controlMode, motor_right);
}