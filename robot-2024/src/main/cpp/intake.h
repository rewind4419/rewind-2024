#pragma once
#include <rev/CANSparkMax.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <frc/DigitalInput.h>
#include "fennec/config.h"

struct Intake
{
    float beam_break_val;
    float intake_speed = 0.0f;
    rev::CANSparkMax* intake_motor;
    //frc::DigitalInput beam_break{CFG_INTAKE_BB_DIO};
};

void initIntake(Intake* intake);
void updateIntake(Intake* intake);
