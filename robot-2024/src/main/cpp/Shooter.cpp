#include "Shooter.h"

void initShooter(Shooter* shooter)
{
    shooter->control_motor = new rev::CANSparkMax(CFG_SHOOTER_CONTROL_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    shooter->firing_motor = new rev::CANSparkMax(CFG_SHOOTER_FIRING_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless);

    shooter->axis_motors[0] = new rev::CANSparkMax(CFG_SHOOTER_AXIS_LEFT, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    shooter->axis_motors[1] = new rev::CANSparkMax(CFG_SHOOTER_AXIS_RIGHT, rev::CANSparkMaxLowLevel::MotorType::kBrushless);

    shooter->deliver_angle_offset = 0;

    float curr_ang = shooter->shooter_encoder.GetAbsolutePosition() - CFG_SHOOTER_AXIS_OFFSET;
    shooter->sum_angle = curr_ang;
}

void updateShooter(Shooter* shooter)
{
    shooter->beam_break_val = shooter->beam_break.Get();

    shooter->control_motor->Set(shooter->control_motor_speed);
    shooter->firing_motor->Set(shooter->firing_motor_speed);


    // Shooter angle code

    float curr_ang_tmp = shooter->shooter_encoder.GetAbsolutePosition() - CFG_SHOOTER_AXIS_OFFSET;

    // Probably not needed, can prob just do current - prev to get difference
    v2 current_facing = rotate({ 0, 1 }, curr_ang_tmp * 2 * M_PI);
    v2 last_facing = rotate({ 0, 1 }, shooter->sum_angle * 2 * M_PI);

    shooter->sum_angle += angleBetween(last_facing, current_facing) / (2 * M_PI);


    float angle_diff = shooter->target_angle + shooter->deliver_angle_offset - shooter->sum_angle;
    float pid = evalPid(&shooter->shooter_pid, angle_diff, CFG_DELTA_TIME);

    float target_throttle = CLAMP(pid, -CFG_SHOOTER_AXIS_THROTTLE, CFG_SHOOTER_AXIS_THROTTLE);

    shooter->axis_throttle = mix(
        shooter->axis_throttle,
        target_throttle,
        0.2f
    );

    shooter->axis_throttle = CLAMP(shooter->axis_throttle, -CFG_SHOOTER_AXIS_THROTTLE, CFG_SHOOTER_AXIS_THROTTLE);

    for(int i = 0; i < CFG_SHOOTER_AXIS_MOTOR_COUNT; i++)
    {
        if (i == 1) shooter->axis_motors[i]->Set(-shooter->axis_throttle);
        else shooter->axis_motors[i]->Set(shooter->axis_throttle);
    }

    
}

