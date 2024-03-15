#include "Shooter.h"
#include "Robot.h"

void initShooter(Shooter* shooter)
{
    shooter->control_motor = new rev::CANSparkFlex(CFG_SHOOTER_CONTROL_MOTOR, rev::CANSparkFlex::MotorType::kBrushless);
    shooter->firing_motor = new rev::CANSparkFlex(CFG_SHOOTER_FIRING_MOTOR, rev::CANSparkFlex::MotorType::kBrushless);

    shooter->axis_motors[0] = new rev::CANSparkFlex(CFG_SHOOTER_AXIS_LEFT, rev::CANSparkFlex::MotorType::kBrushless);
    shooter->axis_motors[1] = new rev::CANSparkFlex(CFG_SHOOTER_AXIS_RIGHT, rev::CANSparkFlex::MotorType::kBrushless);

    shooter->shooter_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(shooter->axis_motors[0]->GetEncoder());

    shooter->deliver_angle_offset = 0;

    shooter->prev_angle = shooter->shooter_encoder->GetPosition();
}

void updateShooter(Shooter* shooter, RobotData* r)
{
    shooter->beam_break_val = shooter->beam_break.Get();

    shooter->control_motor->Set(shooter->control_motor_speed);

    shooter->firing_motor->Set(shooter->firing_motor_speed);

    float shooter_angle = shooter->sum_angle / CFG_SHOOTER_MAX_ANGLE * CFG_SHOOTER_ANGLE_RANGE + CFG_SHOOTER_ANGLE_OFFSET;

    float counter_throttle = CFG_SHOOTER_PERPENDICULAR_THROTTLE * cos(shooter_angle);

    // Shooter angle code
    float curr_angle = shooter->shooter_encoder->GetPosition();

    shooter->sum_angle += curr_angle - shooter->prev_angle;

    float angle_interpol_val = shooter->sum_angle / CFG_SHOOTER_MAX_ANGLE * CFG_SHOOTER_ANGLE_RANGE;

    shooter->target_angle = CLAMP(shooter->target_angle, 0, CFG_SHOOTER_ANGLE_RANGE);

    float target_angle_w_adjustment = shooter->target_angle + ( r->input.mate.joystick_right.y * CFG_SHOOTER_ANGLE_RANGE / 10);

    target_angle_w_adjustment = CLAMP(target_angle_w_adjustment, 0, CFG_SHOOTER_ANGLE_RANGE);

    float inputted_angle;

    float interpol_diff;
    if(shooter->intake_task) inputted_angle =  shooter->target_angle;
    else inputted_angle = target_angle_w_adjustment;


    // inputted_angle = frc::SmartDashboard::GetNumber("Current Angle", 0);

    interpol_diff = inputted_angle - angle_interpol_val;

    printf("Current Angle = %f\n", inputted_angle );


     




    float pid = evalPid(&shooter->shooter_pid, interpol_diff, CFG_DELTA_TIME);
    // printf("PID = %f \n", pid);

    float target_throttle = CLAMP(pid, -CFG_SHOOTER_AXIS_THROTTLE, CFG_SHOOTER_AXIS_THROTTLE);

    shooter->axis_throttle = mix(
        shooter->axis_throttle,
        target_throttle,
        0.8f
    );

    // printf("THROTTLE PRIOR = %f\n", shooter->axis_throttle);

    shooter->axis_throttle += counter_throttle;

    // printf("THROTTLE = %f\n", shooter->axis_throttle);

    shooter->axis_throttle = CLAMP(shooter->axis_throttle, -CFG_SHOOTER_AXIS_THROTTLE, CFG_SHOOTER_AXIS_THROTTLE);


    for(int i = 0; i < CFG_SHOOTER_AXIS_MOTOR_COUNT; i++)
    {
        if (i == 1) shooter->axis_motors[i]->Set(-shooter->axis_throttle);
        else shooter->axis_motors[i]->Set(shooter->axis_throttle);
    }

    shooter->prev_angle = curr_angle;
    
}

void calibrateShooter(Shooter* shooter)
{
    shooter->sum_angle += shooter->shooter_encoder->GetPosition() - shooter->prev_angle;
    shooter->prev_angle = shooter->shooter_encoder->GetPosition();

    printf("Shooter Sum Angle = %f\n", shooter->sum_angle);
}

