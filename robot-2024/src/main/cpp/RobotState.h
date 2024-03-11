#pragma once
#include "fennec/taskmgr.h"
#include "Robot.h"

enum RobotState
{
    INTAKE_OFF_GROUND = 0,

    INTAKE_TRANSFER,

    INTAKE_OFF_GROUND_WITHOUT_BB,

    SHOOTER_STOP,
};

void robotCmd(RobotData* r, RobotState state);
