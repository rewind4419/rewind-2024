#include "RobotState.h"
#include <iostream>
#include "fennec/config.h"

void robotCmd(RobotData* r, RobotState state)
{
    switch(state)
    {
        case INTAKE_OFF_GROUND:
        {

            {
                Task t;
                t.type = TASK_INTAKE_PULLER;
                pushTask(&r->taskmgr, t);
            }
        } break;
        case INTAKE_OFF_GROUND_WITHOUT_BB:
        {
            {
                Task t;
                t.type = TASK_INTAKE_WITHOUT_BB;
                pushTask(&r->taskmgr, t);
            }
        } break;

        case INTAKE_TRANSFER:
        {

            r->shooter.intake_task = true;
            {
                Task t;
                t.type = TASK_SHOOTER_POSITIONING;
                t.shooter.target_angle = 1.2f;
                t.shooter.epsilon = 0.4f;
                pushTask(&r->taskmgr, t);
            }     

            {
                Task t;
                t.type = TASK_SHOOTER_PULLER;
                pushTask(&r->taskmgr, t);
            }

            pushTask(&r->taskmgr, genTaskDelay(0.5));

            {
                Task t;
                t.type = TASK_SHOOTER_POSITIONING;
                t.shooter.target_angle = 0;
                t.shooter.epsilon = 0.4f;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_SEAT_RING;
                t.shooter.delay_timer = 0;
                t.shooter.delay_length = 0.05f;
                t.shooter.seat_speed = -0.8f;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                pushTask(&r->taskmgr, t);
            }
            
        } break;

        case SHOOTER_STOP:
        {
            pushTask(&r->taskmgr, genTaskDelay(1));
            {
                Task t;
                t.type = TASK_SHOOTER_STOP;
                pushTask(&r->taskmgr, t);
            }

        } break;

        case SHOOTER_DELIVER_SPEAKER:
        {
            printf("running speaker task\n");
            {
                Task t;
                t.type = TASK_SHOOTER_POSITIONING;
                t.shooter.target_angle = 0.9f - CFG_SHOOTER_ANGLE_OFFSET;
                t.shooter.epsilon = 0.4f;
                pushTask(&r->taskmgr, t);
            }


        } break;

        case SHOOTER_DELIVER_AMP:
        {
            {
                Task t;
                t.type = TASK_SHOOTER_POSITIONING;
                t.shooter.target_angle = 1.5f;
                t.shooter.epsilon = 0.4f;
                pushTask(&r->taskmgr, t);
            }   

            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                t.firing_motor.direction = -0.4f;
                pushTask(&r->taskmgr, t);
            }

        } break;

        case ANGLE_TO_SPEAKER:
        {
            r->photon.first_aim = false;
            {
                Task t;
                t.type = TASK_ANGLE_TO_TAG;
                t.photon_aligner.align_tag_id = 8;
                pushTask(&r->taskmgr, t);
            }

        } break;

        
    }   
    
}