#pragma once
#include <rev/CANSparkMax.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <frc/DigitalInput.h>

struct RobotData;

struct Intake
{
    float beam_break_val;
    float intake_speed = 0;
    rev::CANSparkMax* intake_motor;
    frc::DigitalInput input{CFG_BB_DIO};
};

void initIntake(Intake* intake);
void updateIntake(Intake* intake);