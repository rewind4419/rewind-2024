#pragma once
#include <rev/CANSparkMax.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/DigitalInput.h>
#include "fennec/config.h"
#include "fennec/pid.h"

struct Shooter
{
    float beam_break_val;
    float control_motor_speed = 0;
    float firing_motor_speed = 0;
    float deliver_angle_offset = 0;
    float sum_angle = 0;
    float target_angle = CFG_SHOOTER_AXIS_OFFSET;
    float axis_throttle;

    PID shooter_pid = { .kP = 1.25f, .kI = 0, .kD = 0,  .errorAccum = 0, .lastError = 0 };

    frc::DutyCycleEncoder shooter_encoder { CFG_SHOOTER_ENCODER };
    rev::CANSparkMax* axis_motors[CFG_SHOOTER_AXIS_MOTOR_COUNT];
    rev::CANSparkMax* control_motor;
    rev::CANSparkMax* firing_motor;
    frc::DigitalInput beam_break{CFG_SHOOTER_BB_DIO};
};

void initShooter(Shooter* shooter);
void updateShooter(Shooter* shooter);

