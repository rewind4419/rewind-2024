#include "Elevator.h"
#include "Robot.h"
#include <stdio.h>

void initElevator(Elevator* elevator)
{
    elevator->left_motor = new rev::CANSparkMax(CFG_ELEVATOR_LEFT_MOTOR, rev::CANSparkMax::MotorType::kBrushless);
    elevator->right_motor = new rev::CANSparkMax(CFG_ELEVATOR_RIGHT_MOTOR, rev::CANSparkMax::MotorType::kBrushless);

    //In this case, the height represents the angle recorded from the encoder that caused the elevator to go up or down
    // elevator->target_height = CFG_ELEVATOR_MIN_ROTATION
    elevator->min_encoder_value = elevator->elevator_encoder.GetAbsolutePosition();
    elevator->max_encoder_value = elevator->min_encoder_value + 2.1;
    elevator->prev_height = elevator->elevator_encoder.GetAbsolutePosition();
    printf("curr rot = %f\n", elevator->min_encoder_value);
    printf("init prev = %f\n", elevator->prev_height);

    elevator->target_height = 0;

}

void resetElevator( Elevator* elevator)
{
    elevator->target_height = 0;
}

void updateElevator(Elevator* elevator, RobotData* r)
{
    if(elevator->test_first_frame)
    {
        if(elevator->elevator_encoder.GetAbsolutePosition() < 0)
        {
            elevator->reverse = true;
            elevator->prev_height = 1 + elevator->elevator_encoder.GetAbsolutePosition();
        }
        else 
        {
            elevator->prev_height = elevator->elevator_encoder.GetAbsolutePosition();
            elevator->reverse = false;
        }
        elevator->test_first_frame = false;
    }

    float curr_height;

    if(elevator->reverse)
    {
        curr_height = 1 + elevator->elevator_encoder.GetAbsolutePosition();
    }
    else curr_height = elevator->elevator_encoder.GetAbsolutePosition();
    
    float rot_delta = -1 * (curr_height - elevator->prev_height);

    if(rot_delta < -0.5f) elevator->sum_rotation += 1;
    else if(rot_delta > 0.5) elevator->sum_rotation -= 1;

    elevator->sum_rotation += rot_delta;
    elevator->prev_height = curr_height;

    float current_height = (elevator->sum_rotation) / CFG_ELEVATOR_MAX_ROTATION * CFG_ELEVATOR_RANGE;

    elevator->target_height = CLAMP(elevator->target_height, 0, 0.4f );

    float target_diff = elevator->target_height - current_height;

    frc::SmartDashboard::PutNumber("Current Height", current_height);

    float pid = evalPid(&elevator->elevator_pid, target_diff, CFG_DELTA_TIME);

    float ideal_throttle = CLAMP(pid, -CFG_ELEVATOR_THROTTLE, CFG_ELEVATOR_THROTTLE);

    elevator->curr_throttle = mix(
        elevator->curr_throttle,
        ideal_throttle,
        0.2f //Change
    );

    elevator->curr_throttle = CLAMP(elevator->curr_throttle, -CFG_ELEVATOR_THROTTLE, CFG_ELEVATOR_THROTTLE);
    
    frc::SmartDashboard::PutNumber("Elevator Throttle", elevator->curr_throttle);
    elevator->right_motor->Set(-elevator->curr_throttle);
    elevator->left_motor->Set(-elevator->curr_throttle);
    frc::SmartDashboard::PutNumber("Elevator set power", elevator->curr_throttle);
}

void calibrateElevator(Elevator* elevator)
{   
    // Elevator angle calculation
    float curr_height = -elevator->elevator_encoder.GetAbsolutePosition();
    float rot_delta = curr_height - elevator->prev_height;

    if(rot_delta < -0.5f) elevator->sum_rotation += 1;
    else if(rot_delta > 0.5) elevator->sum_rotation -= 1;

    elevator->sum_rotation += rot_delta;
    elevator->prev_height = curr_height;

    printf("Elevator Height = %f\n", elevator->sum_rotation);
}

