#pragma once
#include <rev/CANSparkMax.h>
#include <rev/CANSparkFlex.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/DigitalInput.h>
#include "fennec/config.h"
#include "fennec/pid.h"
#include "fennec/maths.h"
#include <numbers>

struct RobotData;

struct Elevator
{
    float left_motor_speed = 0;
    float right_motor_speed = 0;
    float sum_rotation = 0;
    float prev_height;
    float target_height = 0;
    float curr_throttle;

    bool intake_assignment = false;

    frc::DutyCycleEncoder elevator_encoder{CFG_ELEVATOR_ENCODER};

    PID elevator_pid = { .kP = 20.0f, .kI = 0, .kD = 0,  .errorAccum = 0, .lastError = 0 };

    float min_encoder_value;
    float max_encoder_value;
    
    rev::CANSparkMax* left_motor;
    rev::CANSparkMax* right_motor;

    bool test_first_frame = true;
    float elevator_timer = 0;
    bool reverse = false;
};

void initElevator(Elevator* elevator);
void updateElevator(Elevator* elevator, RobotData* r);
void calibrateElevator(Elevator* elevator);









//---------------------------------------------------------------------------------------------------------------
/*
Previous shooter code in struct Shooter:
    bool beam_break_val;
    float deliver_angle_offset = 0;
    float target_angle = CFG_SHOOTER_START_ANGLE;
    float axis_throttle;
    float shooter_delay_timer = 0;

    bool firing_motor_task = false;
    bool shooter_first_time = true;
    bool intake_task = false;

    PID shooter_pid = { .kP = 0.9f, .kI = 0, .kD = 0,  .errorAccum = 0, .lastError = 0 };

    rev::CANSparkFlex* axis_motors[CFG_SHOOTER_AXIS_MOTOR_COUNT];

    frc::DigitalInput beam_break{CFG_SHOOTER_BB_DIO};

    rev::SparkMaxRelativeEncoder elevator_encoder = left_motor->GetEncoder().GetPosition;
    robot->elevator_encoder.SetDistancePerPulse(1.0 / 360.0 * 2.0 * numbers::pi * 1.5)
*/