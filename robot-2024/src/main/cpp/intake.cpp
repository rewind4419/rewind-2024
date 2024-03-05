#include "intake.h"
#include "robot.h"


void initIntake(Intake* intake)
{
    intake->intake_motor = new rev::CANSparkMax(CFG_INTAKE_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
}

void updateIntake(Intake* intake)
{
    intake->beam_break_val = intake->input.Get();
    intake->intake_motor->Set(intake->intake_speed);
}