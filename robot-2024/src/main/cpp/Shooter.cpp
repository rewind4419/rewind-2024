#include "Shooter.h"

void initShooter(Shooter* shooter)
{
    shooter->control_motor = new rev::CANSparkMax(CFG_SHOOTER_CONTROL_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    shooter->firing_motor = new rev::CANSparkMax(CFG_SHOOTER_FIRING_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless);

    shooter->axis_motors[0] = new rev::CANSparkFlex(CFG_SHOOTER_AXIS_LEFT, rev::CANSparkFlex::MotorType::kBrushless);
    shooter->axis_motors[1] = new rev::CANSparkFlex(CFG_SHOOTER_AXIS_RIGHT, rev::CANSparkFlex::MotorType::kBrushless);

    shooter->shooter_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(shooter->axis_motors[0]->GetEncoder());

    shooter->deliver_angle_offset = 0;

    float curr_ang = shooter->shooter_encoder->GetPosition();
    shooter->sum_angle += curr_ang;
}

void updateShooter(Shooter* shooter)
{
    shooter->beam_break_val = shooter->beam_break.Get();

    shooter->control_motor->Set(shooter->control_motor_speed);
    shooter->firing_motor->Set(shooter->firing_motor_speed);


    // Shooter angle code

    shooter->sum_angle += shooter->shooter_encoder->GetPosition();

    float angle_interpol_val = shooter->sum_angle / CFG_SHOOTER_MAX_ANGLE;
    float interpol_diff = shooter->target_angle - shooter->sum_angle;

    float pid = evalPid(&shooter->shooter_pid, interpol_diff, CFG_DELTA_TIME);

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

