#pragma once
#include <rev/CANSparkMax.h>
#include <rev/CANSparkFlex.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/DigitalInput.h>
#include "fennec/config.h"
#include "fennec/pid.h"

struct RobotData;

struct Shooter
{
    bool beam_break_val;
    float control_motor_speed = 0;
    float firing_motor_speed = 0;
    float deliver_angle_offset = 0;
    float sum_angle = 0;
    float target_angle = CFG_SHOOTER_START_ANGLE;
    float axis_throttle;
    float prev_angle;
    float shooter_delay_timer = 0;
    float auto_await_timeout = 0.0;

    float auto_beam_break_timer = 0.0;

    double last_firing_motor_target = 0.0;
    // ^ Used to detect when the target speed has changed to reset integral

    float shooter_firing_calibrate_speed;

    bool firing_motor_task = false;
    bool shooter_first_time = true;
    bool intake_task = false;
    bool firing_mode = false;
    bool brake = false;
    bool amp_mode = false;

    bool trigger_or_bumper_first = true;



    PID shooter_pid = { .kP = 0.4f, .kI = 0, .kD = 0,  .errorAccum = 0, .lastError = 0 };

    PID amp_wheel_pid = { .kP = 0.00025f, .kI = 0.0001, .kD = 0,  .errorAccum = 0, .lastError = 0 };

    //PID firing_wheel_pid = { .kP = 0.001f, .kI = 0.000235f, .kD = 0.0,  .errorAccum = 0, .lastError = 0 };
    PID firing_wheel_pid = { .kP = 0.0f, .kI = 0.0002f, .kD = 0.0,  .errorAccum = 0, .lastError = 0 };
    // PID firing_wheel_pid = { .kP = 0.0f, .kI = 0.000001f, .kD = 0.00000,  .errorAccum = 0, .lastError = 0 };

    


    std::unique_ptr<rev::SparkMaxRelativeEncoder> shooter_encoder;
    std::unique_ptr<rev::SparkMaxRelativeEncoder> firing_encoder;
    std::unique_ptr<rev::SparkMaxRelativeEncoder> control_encoder;
    
    rev::CANSparkFlex* axis_motors[CFG_SHOOTER_AXIS_MOTOR_COUNT];
    rev::CANSparkFlex* control_motor;
    rev::CANSparkFlex* firing_motor;
    rev::CANSparkFlex* firing_motor_2;
    frc::DigitalInput beam_break{CFG_SHOOTER_BB_DIO};
    bool beam_break_enabled = true;
};

void initShooter(Shooter* shooter);
void updateShooter(Shooter* shooter, RobotData* r);
void calibrateShooterAngle(Shooter* shooter);
void calibrateShooterFiringMotor(Shooter* shooter);

void resetShooter( Shooter* shooter);




