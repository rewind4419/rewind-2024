#pragma once
#include "fennec/taskmgr.h"
#include "Robot.h"

enum RobotState
{
    STATE_NONE = 0,

    INTAKE_OFF_GROUND,

    INTAKE_TRANSFER,

    INTAKE_OFF_GROUND_WITHOUT_BB,

    SHOOTER_STOP,

    SHOOTER_DELIVER_SPEAKER,

    SHOOTER_DELIVER_AMP,

    ANGLE_TO_SPEAKER,
};


void robotCmd(RobotData* r, RobotState state);
