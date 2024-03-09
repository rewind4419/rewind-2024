#include "fennec/taskmgr.h"

enum RobotState
{
    INTAKE_OFF_GROUND = 0,

    INTAKE_TRANSFER,

    INTAKE_OFF_GROUND_WITHOUT_BB


};

void robotCmd(TaskMgr* mgr, RobotState state);
