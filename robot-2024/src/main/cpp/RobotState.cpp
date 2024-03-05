#include "RobotState.h"
#include "fennec/config.h"

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
            {
                Task t;
                t.type = TASK_SHOOTER_POSITIONING;
                t.shooter.target_angle = 20.0f / CFG_SHOOTER_MAX_ANGLE;
                pushTask(mgr, t);
            }

            {
                Task t;
                t.type = TASK_SHOOTER_PULLER;
                pushTask(mgr, t);
            }

            pushTask(mgr, genTaskDelay(0.5));

            {
                Task t;
                t.type = TASK_SHOOTER_POSITIONING;
                t.shooter.target_angle = 0 / CFG_SHOOTER_MAX_ANGLE;
                pushTask(mgr, t);
            }

        }
    }   
    
}