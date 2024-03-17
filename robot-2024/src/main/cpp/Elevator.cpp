#include "Elevator.h"
#include "Robot.h"
#include <stdio.h>

void initElevator(Elevator* elevator)
{
    elevator->left_motor = new rev::CANSparkMax(CFG_ELEVATOR_LEFT_MOTOR, rev::CANSparkMax::MotorType::kBrushless);
    elevator->right_motor = new rev::CANSparkMax(CFG_ELEVATOR_RIGHT_MOTOR, rev::CANSparkMax::MotorType::kBrushless);

    //In this case, the height represents the angle recorded from the encoder that caused the elevator to go up or down
    elevator->prev_height = elevator->elevator_encoder.GetAbsolutePosition();
}

void updateElevator(Elevator* elevator, RobotData* r)
{
    // Elevator angle calculation
    float curr_height = elevator->elevator_encoder->GetAbsolutePosition();
    elevator->sum_rotation += curr_height - elevator->prev_height;
    elevator->prev_height = curr_height;

    float current_height = elevator->sum_rotation / CFG_ELEVATOR_MAX_ROTATION * CFG_ELEVATOR_RANGE;

    elevator->target_height = CLAMP(elevator->target_height, 0, CFG_ELEVATOR_RANGE);

    float target_diff = elevator->target_height - current_height;

    float pid = evalPid(&elevator->elevator_pid, target_diff, CFG_DELTA_TIME);

    float ideal_throttle = CLAMP(pid, -CFG_ELEVATOR_THROTTLE, CFG_ELEVATOR_THROTTLE);

    elevator->curr_throttle = mix(
        elevator->curr_throttle,
        ideal_throttle,
        0.8f //Change
    );

    elevator->curr_throttle = CLAMP(elevator->curr_throttle, -CFG_ELEVATOR_THROTTLE, CFG_ELEVATOR_THROTTLE);

    elevator->left_motor->Set(elevator->curr_throttle);
    elevator->right_motor->Set(elevator->curr_throttle);

}

void calibrateElevator(Elevator* elevator)
{
    // Elevator angle calculation
    float curr_height = elevator->elevator_encoder->GetAbsolutePosition();
    elevator->sum_rotation += curr_height - elevator->prev_height;
    elevator->prev_height = curr_height;

    printf("Elevator Height = %f\n", elevator->sum_rotation);
}








//----------------------------------------------------------------------------------------------------------------

/*
Previous code from the shooter.cpp file:
    initshooter:
        shooter->axis_motors[0] = new rev::CANSparkFlex(CFG_SHOOTER_AXIS_LEFT, rev::CANSparkFlex::MotorType::kBrushless);
        shooter->axis_motors[1] = new rev::CANSparkFlex(CFG_SHOOTER_AXIS_RIGHT, rev::CANSparkFlex::MotorType::kBrushless);
        shooter->deliver_angle_offset = 0;
    updateshooter:
        shooter->beam_break_val = shooter->beam_break.Get();
        float shooter_angle = shooter->sum_angle / CFG_SHOOTER_MAX_ANGLE * CFG_SHOOTER_ANGLE_RANGE + CFG_SHOOTER_ANGLE_OFFSET;
        float counter_throttle = CFG_SHOOTER_PERPENDICULAR_THROTTLE * cos(shooter_angle);
        float angle_interpol_val = shooter->sum_angle / CFG_SHOOTER_MAX_ANGLE * CFG_SHOOTER_ANGLE_RANGE;
        shooter->target_angle = CLAMP(shooter->target_angle, 0, CFG_SHOOTER_ANGLE_RANGE);
        float target_angle_w_adjustment = shooter->target_angle + ( r->input.mate.joystick_right.y * CFG_SHOOTER_ANGLE_RANGE / 10);
        target_angle_w_adjustment = CLAMP(target_angle_w_adjustment, 0, CFG_SHOOTER_ANGLE_RANGE);
        float inputted_angle;
        float interpol_diff;
        if(shooter->intake_task) inputted_angle =  shooter->target_angle;
        else inputted_angle = target_angle_w_adjustment;
        inputted_angle = frc::SmartDashboard::GetNumber("Current Angle", 0);
        interpol_diff = inputted_angle - angle_interpol_val;
        printf("Current Angle = %f\n", inputted_angle );
        float pid = evalPid(&shooter->shooter_pid, interpol_diff, CFG_DELTA_TIME);
        printf("PID = %f \n", pid);
        float target_throttle = CLAMP(pid, -CFG_SHOOTER_AXIS_THROTTLE, CFG_SHOOTER_AXIS_THROTTLE);
        shooter->axis_throttle = mix(
            shooter->axis_throttle,
            target_throttle,
            0.8f
        );
        printf("THROTTLE PRIOR = %f\n", shooter->axis_throttle);
        shooter->axis_throttle += counter_throttle;
        printf("THROTTLE = %f\n", shooter->axis_throttle);
        shooter->axis_throttle = CLAMP(shooter->axis_throttle, -CFG_SHOOTER_AXIS_THROTTLE, CFG_SHOOTER_AXIS_THROTTLE);
        for(int i = 0; i < CFG_SHOOTER_AXIS_MOTOR_COUNT; i++)
        {
            if (i == 1) shooter->axis_motors[i]->Set(-shooter->axis_throttle);
            else shooter->axis_motors[i]->Set(shooter->axis_throttle);
        }
*/