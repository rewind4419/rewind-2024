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


    shooter->deliver_angle_offset = 0;

    shooter->prev_angle = shooter->shooter_encoder->GetPosition();

    shooter->shooter_firing_calibrate_speed = 1.0f;
}

void updateShooter(Shooter* shooter, RobotData* r)
{
    // shooter->beam_break_val = shooter->beam_break.Get();

    // Method to get velocity
    // shooter->firing_encoder->GetVelocity();

    float delta_throttle = shooter->firing_motor_speed - shooter->firing_motor_prev_throttle;
    shooter->firing_motor_prev_throttle = shooter->firing_motor_speed;
    
    float firing_curr_angle = shooter->firing_encoder->GetPosition();
    float delta_firing_angle = firing_curr_angle - shooter->firing_prev_angle;
    shooter->firing_prev_angle = firing_curr_angle;

    // printf("delta angle = %f\n", delta_firing_angle);


    // if(fabs(delta_throttle) > 0.9) 
    // {
    //     shooter->brake = true;
    //     printf("brake started\n");
    // }


    float firing_throttle;

    float firing_motor_velocity = shooter->firing_encoder->GetVelocity();

    frc::SmartDashboard::PutNumber("Current Firing Motor Velocity", firing_motor_velocity);


    if(fabs(firing_motor_velocity) < 400.0f)
    {
        shooter->brake = false;
    }
    if(shooter->brake)
    {
        


        if(firing_motor_velocity < 0)
        {
            firing_throttle = 1;
        }
        else if(firing_motor_velocity > 0)
        {
            firing_throttle = -1;
        }
        // printf("braking\n");

    }
    else
    {
        firing_throttle = shooter->firing_motor_speed;
    }
    // printf("firing throttle = %f\n", firing_throttle);

    // frc::SmartDashboard::PutNumber("RPM", shooter->firing_motor.Get);

    shooter->firing_motor->Set(firing_throttle);

    shooter->control_motor->Set(-1 * shooter->control_motor_speed);

    float shooter_angle = shooter->sum_angle / CFG_SHOOTER_MAX_ANGLE * CFG_SHOOTER_ANGLE_RANGE + CFG_SHOOTER_ANGLE_OFFSET;

    float counter_throttle = CFG_SHOOTER_PERPENDICULAR_THROTTLE * cos(shooter_angle);

    float elevator_dist = (r->elevator.sum_rotation / CFG_ELEVATOR_MAX_ROTATION * CFG_ELEVATOR_RANGE);

    float cg_hypo = sqrtf(std::pow(4.5 * INCH_TO_METER, 2) + std::pow(elevator_dist + 14 * INCH_TO_METER, 2) );

    float cg_extension_coeff = cg_hypo / (14.705 * INCH_TO_METER);
    counter_throttle *= cg_extension_coeff;

    frc::SmartDashboard::PutNumber("counter#2", cg_extension_coeff);

    // Shooter angle code
    float curr_angle = shooter->shooter_encoder->GetPosition();

    shooter->sum_angle += curr_angle - shooter->prev_angle;

    float angle_interpol_val = shooter->sum_angle / CFG_SHOOTER_MAX_ANGLE * CFG_SHOOTER_ANGLE_RANGE;

    frc::SmartDashboard::PutNumber("Shooter Current Angle", angle_interpol_val);
    frc::SmartDashboard::PutNumber("Shooter Target Angle", shooter->target_angle);

    shooter->target_angle = CLAMP(shooter->target_angle, 0, 1.6);

    if(r->)
    float target_angle_w_adjustment = shooter->target_angle + ( r->input.mate.joystick_right.y * CFG_SHOOTER_ANGLE_RANGE / 10);

    target_angle_w_adjustment = CLAMP(target_angle_w_adjustment, 0, CFG_SHOOTER_ANGLE_RANGE);

    float inputted_angle;

    float interpol_diff;
    if(shooter->intake_task) inputted_angle =  shooter->target_angle;
    else inputted_angle = target_angle_w_adjustment;


    // inputted_angle = frc::SmartDashboard::GetNumber("Current Angle", 0);

    interpol_diff = inputted_angle - angle_interpol_val;

    frc::SmartDashboard::PutNumber("Shooter Diff Angle", interpol_diff);


    // printf("Current Angle = %f\n", inputted_angle );


    float pid = evalPid(&shooter->shooter_pid, interpol_diff, CFG_DELTA_TIME);
    // printf("PID = %f \n", pid);

    frc::SmartDashboard::PutNumber("Shooter PID error accum", shooter->shooter_pid.errorAccum);
    frc::SmartDashboard::PutNumber("Shooter PID last error", shooter->shooter_pid.lastError);
frc::SmartDashboard::PutNumber("Shooter PID kool p", shooter->shooter_pid.kP);
frc::SmartDashboard::PutNumber("Shooter PID koolaid i", shooter->shooter_pid.kI);
frc::SmartDashboard::PutNumber("Shooter PID d", shooter->shooter_pid.kD);


    frc::SmartDashboard::PutNumber("Shooter PID", pid);


    float target_throttle = CLAMP(pid, -CFG_SHOOTER_AXIS_THROTTLE, CFG_SHOOTER_AXIS_THROTTLE);

    frc::SmartDashboard::PutNumber(" PID  After clamp", target_throttle);


    shooter->axis_throttle = mix(
        shooter->axis_throttle,
        target_throttle,
        0.9f
    );

    frc::SmartDashboard::PutNumber("PID after mix", shooter->axis_throttle);


    // printf("THROTTLE PRIOR = %f\n", shooter->axis_throttle);

    shooter->axis_throttle += counter_throttle;
    frc::SmartDashboard::PutNumber("PID after counter", shooter->axis_throttle);
    frc::SmartDashboard::PutNumber("Counter Throttle", counter_throttle);


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
