#include "Intake.h"
#include "fennec/taskmgr.h"

void initIntake(Intake* intake)
{
    intake->intake_motor = new rev::CANSparkMax(CFG_INTAKE_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
}

void updateIntake(Intake* intake)
{
    intake->beam_break_val = intake->beam_break.Get();
    intake->intake_motor->Set(intake->intake_speed);
}

