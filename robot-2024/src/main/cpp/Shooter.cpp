#include "Shooter.h"
#include "Robot.h"

void initShooter(Shooter* shooter)
{
    shooter->control_motor = new rev::CANSparkFlex(CFG_SHOOTER_CONTROL_MOTOR, rev::CANSparkFlex::MotorType::kBrushless);
    shooter->firing_motor = new rev::CANSparkFlex(CFG_SHOOTER_FIRING_MOTOR, rev::CANSparkFlex::MotorType::kBrushless);

    shooter->axis_motors[0] = new rev::CANSparkFlex(CFG_SHOOTER_AXIS_LEFT, rev::CANSparkFlex::MotorType::kBrushless);
    shooter->axis_motors[1] = new rev::CANSparkFlex(CFG_SHOOTER_AXIS_RIGHT, rev::CANSparkFlex::MotorType::kBrushless);

    shooter->shooter_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(shooter->axis_motors[0]->GetEncoder());
    shooter->firing_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(shooter->firing_motor->GetEncoder());
    shooter->control_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(shooter->control_motor->GetEncoder());


    shooter->deliver_angle_offset = 0;

    shooter->target_angle = 0;

    shooter->prev_angle = shooter->shooter_encoder->GetPosition();

    shooter->shooter_firing_calibrate_speed = 1.0f;
    shooter->amp_mode = false;

    shooter->beam_break_enabled = true;
}

void resetShooter( Shooter* shooter)
{
    shooter->target_angle = 0;
    shooter->amp_mode = false;
    shooter->beam_break_enabled = true;
}

void updateShooter(Shooter* shooter, RobotData* r)
{
    // shooter->beam_break_val = shooter->beam_break.Get();

    float firing_throttle;
    float firing_motor_velocity = shooter->firing_encoder->GetVelocity();
    frc::SmartDashboard::PutNumber("Current Firing Motor Velocity", firing_motor_velocity);

    //////// Firing Motor Brake Code ////////
    if(fabs(firing_motor_velocity) < 400.0f) shooter->brake = false;
    if(shooter->firing_mode && !r->ready_fire_amp)
    {
        float target_velocity = shooter->firing_motor_speed * CFG_TARGET_VELOCITY_FIRING_WHEELS;
        float current_velocity = shooter->firing_encoder->GetVelocity();
        firing_throttle = evalPid(&shooter->firing_wheel_pid, target_velocity - current_velocity, CFG_DELTA_TIME);
        frc::SmartDashboard::PutNumber("Firing Velocity Diff", target_velocity - current_velocity);
        frc::SmartDashboard::PutNumber("Firing Error Accum", shooter->firing_wheel_pid.errorAccum);
    }
    else 
    {
        shooter->firing_wheel_pid.errorAccum = 0.0;
        firing_throttle = shooter->firing_motor_speed;
    }
    
    if(shooter->brake)
    {

        if(firing_motor_velocity < 0) firing_throttle = 1;
        
        else if(firing_motor_velocity > 0) firing_throttle = -1;
    }

    if(!shooter->firing_mode && !r->ready_fire_amp)
    {
        // shooter->control_motor->SetSmartCurrentLimit(32);
        // shooter->control_motor->SetVoltage;
    }
    else shooter->control_motor->SetSmartCurrentLimit(80);


    // if(shooter->firing_mode)
    // {
    //     float target_velocity = shooter->firing_motor_speed * CFG_TARGET_VELOCITY_FIRING_WHEELS;
    //     float current_velocity = shooter->firing_encoder->GetVelocity();
    //     firing_throttle = evalPid(&shooter->firing_wheel_pid, target_velocity - current_velocity, CFG_DELTA_TIME);
    //     frc::SmartDashboard::PutNumber("Firing Velocity Diff", target_velocity - current_velocity);
    // }
    // else firing_throttle = r->shooter.firing_motor_speed;

    frc::SmartDashboard::PutNumber("Control Velocity", shooter->control_encoder->GetVelocity());
    frc::SmartDashboard::PutNumber("Firing Velocity", shooter->firing_encoder->GetVelocity());
    frc::SmartDashboard::PutNumber("Firing Motor Throttle", firing_throttle);

    if (shooter->amp_mode)
    {
        shooter->control_motor->Set(evalPid(&shooter->amp_wheel_pid, CFG_SHOOTER_AMP_SCORE_TARGET_VELOCITY - shooter->control_encoder->GetVelocity(), CFG_DELTA_TIME));
    }
    else
    {
        shooter->amp_wheel_pid.errorAccum = 0.0;
        shooter->control_motor->Set(-1 * shooter->control_motor_speed);
    }

    shooter->firing_motor->Set(firing_throttle);


    float shooter_angle = shooter->sum_angle / CFG_SHOOTER_MAX_ANGLE * CFG_SHOOTER_ANGLE_RANGE + CFG_SHOOTER_ANGLE_OFFSET;

    //////// Gravitational Feed Forward Code ////////
    float counter_throttle = CFG_SHOOTER_PERPENDICULAR_THROTTLE * cos(shooter_angle);
    float elevator_dist = (r->elevator.sum_rotation / CFG_ELEVATOR_MAX_ROTATION * CFG_ELEVATOR_RANGE);
    float cg_hypo = sqrtf(std::pow(4.5 * INCH_TO_METER, 2) + std::pow(elevator_dist + 14 * INCH_TO_METER, 2) );
    float cg_extension_coeff = cg_hypo / (14.705 * INCH_TO_METER);
    counter_throttle *= cg_extension_coeff;

    //////// Finding Total Angle ////////
    float curr_angle = shooter->shooter_encoder->GetPosition();
    shooter->sum_angle += curr_angle - shooter->prev_angle;

    //////// Pivot Throttle Estimation ////////
    float angle_interpol_val = shooter->sum_angle / CFG_SHOOTER_MAX_ANGLE * CFG_SHOOTER_ANGLE_RANGE;
    frc::SmartDashboard::PutNumber("Current Shooter Pivot Angle", angle_interpol_val);
    shooter->target_angle = CLAMP(shooter->target_angle, 0, CFG_SHOOTER_ANGLE_RANGE);
    
    float target_angle_w_adjustment = shooter->target_angle + ( r->input.mate.joystick_right.y * CFG_SHOOTER_ANGLE_RANGE / 10);
    target_angle_w_adjustment = CLAMP(target_angle_w_adjustment, 0, CFG_SHOOTER_ANGLE_RANGE);

    float inputted_angle;
    if(shooter->intake_task) inputted_angle =  shooter->target_angle;
    else inputted_angle = target_angle_w_adjustment;

    float interpol_diff = inputted_angle - angle_interpol_val;
    float pid = evalPid(&shooter->shooter_pid, interpol_diff, CFG_DELTA_TIME);
    float target_throttle = CLAMP(pid, -CFG_SHOOTER_AXIS_THROTTLE, CFG_SHOOTER_AXIS_THROTTLE);

    shooter->axis_throttle = mix(
        shooter->axis_throttle,
        target_throttle,
        0.9f
    );

    shooter->axis_throttle += counter_throttle;
    shooter->axis_throttle = CLAMP(shooter->axis_throttle, -CFG_SHOOTER_AXIS_THROTTLE, CFG_SHOOTER_AXIS_THROTTLE);

    frc::SmartDashboard::PutNumber("Shooter Throttle", shooter->axis_throttle);

    for(int i = 0; i < CFG_SHOOTER_AXIS_MOTOR_COUNT; i++)
    {
        if (i == 1) shooter->axis_motors[i]->Set(-shooter->axis_throttle);
        else shooter->axis_motors[i]->Set(shooter->axis_throttle);
    }

    shooter->prev_angle = curr_angle;
}

void calibrateShooterAngle(Shooter* shooter)
{
    shooter->sum_angle += shooter->shooter_encoder->GetPosition() - shooter->prev_angle;
    shooter->prev_angle = shooter->shooter_encoder->GetPosition();

    printf("Shooter Sum Angle = %f\n", shooter->sum_angle);
}


void calibrateShooterFiringMotor(Shooter* shooter)
{
    shooter->firing_motor->Set(shooter->firing_motor_speed);
    shooter->control_motor->Set(-1 * shooter->control_motor_speed);
    frc::SmartDashboard::PutNumber("Current Firing Motor Throttle", shooter->firing_motor_speed);
    frc::SmartDashboard::PutNumber("Current Control Motor Throttle", -shooter->control_motor_speed);
}
