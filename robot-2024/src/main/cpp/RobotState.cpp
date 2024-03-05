#include "RobotState.h"

void robotCmd(TaskMgr* mgr, RobotState state)
{
    switch(state)
    {
        case INTAKE_OFF_GROUND:
        {
            {
                Task t;
                t.type = TASK_INTAKE_PULLER;
                pushTask(mgr, t);
            }
        } break;

        case INTAKE_TRANSFER:
        {
            

        }
    }   
    
}