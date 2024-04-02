#include "RobotState.h"
#include <iostream>
#include "fennec/config.h"
#include "Robot.h"

/*
SDR Auto issues

1. Auto missing side shots
- Fix tuning? risky
- Move points to middle? possible, needs testing

2. What if break beam fails? How to fix it in a match?
- Add FRC Dashsboard option to disable breakbeam
    - Need to test FRC Dashboard for that
    - Can use it for auto too if its tested
- Add a thing that automatically disables beam break if its already false
^ DOING THIS
(it didn't work, just leaving it for now)

3. IMU recalibration to fix field centric
- Make sure robot is stable on init
- Run factory calibration again once


*/

void robotCmd(RobotData* r, RobotCommand state)
{
    // r->lastCalledState = state;

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
                t.type = TASK_CHECK_INTAKE_FREE;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_SHOOTER_PULLER;
                pushTask(&r->taskmgr, t);
            }

            // pushTask(&r->taskmgr, genTaskDelay(0.5));

            // {
            //     Task t;
            //     t.type = TASK_SEAT_RING;
            //     t.shooter.delay_timer = 0;
            //     t.shooter.delay_length = 0.05f;
            //     t.shooter.seat_speed_control = -0.8f;
            //     t.shooter.seat_speed_firing = 0.0f;
            //     pushTask(&r->taskmgr, t);
            // }

            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_SHOOTER_POSITIONING;
                t.shooter.target_angle = 0;
                t.shooter.epsilon = 0.4f;
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
            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                t.firing_motor.direction = 1;
                pushTask(&r->taskmgr, t);
            }

        } break;

        case SHOOTER_DELIVER_AMP:
        {
            
            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                t.firing_motor.direction = -0.5f;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_SHOOTER_POSITIONING;
                t.shooter.target_angle = 1.45f;
                // t.shooter.target_angle = 1.4f;
                t.shooter.epsilon = 0.4f;
                pushTask(&r->taskmgr, t);
            }   

            {
                Task t;
                t.type = TASK_ELEVATOR_POSITIONING;
                t.elevator.target_height = 0.4f;
                t.elevator.epsilon = 0.2f;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_AMP_READY;
                pushTask(&r->taskmgr, t);
            }



        } break;

        case ANGLE_TO_SPEAKER:
        {
            int tag_number;
            if(r->side == 0) tag_number = 7;
            if(r->side == 1) tag_number = 4;
            // Use id 7 for blue side and 4 for red side
            r->photon.first_aim = false;
            {
                Task t;
                t.type = TASK_ANGLE_TO_TAG;
                t.photon_aligner.align_tag_id = tag_number;
                pushTask(&r->taskmgr, t);
            }

        } break;

        //Comment out if it doesn't work - Nethra
        case CLIMB_POSITIONING:
        {
            {
                printf("Quein  forst on|\n");
                Task t;
                t.type = TASK_ANGLE_FOR_CLIMB;
                t.shooter.target_angle = 1.25f;
                t.shooter.epsilon = 0.05f;
                pushTask(&r->taskmgr, t);
            }      

            {
                printf("Queued 2nd\n");
                Task t;
                t.type = TASK_EXTEND_FOR_CLIMB;
                t.elevator.ideal_height = 0.275f; //Change
                t.elevator.epsilon = 0.1f;
                pushTask(&r->taskmgr, t);
            }
        } break;

        //Comment out if it doesn't work - Nethra
        case CLIMBING:
        {
            {
                Task t;
                t.type = TASK_RETRACT_FOR_CLIMB;
                t.elevator.retract_height = 0.0f;
                t.elevator.epsilon = 0.05f;
                pushTask(&r->taskmgr, t);
            }
        } break;
    }   
    
}


void autoCmd(RobotData* r, AutoState state, AutoAlliance alliance)
{
    switch(state)
    {
        case AUTO_4_PIECE:
        {
            frc::SmartDashboard::PutNumber("Auto init Delay", r->auto_init_delay);
            pushTask(&r->taskmgr, genTaskDelay(r->auto_init_delay));

            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                t.firing_motor.direction = 0;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.target_pose = BLUE_NOTE_PREPICKUP_LEFT;
                t.waypoint.epsilon = 0.2f;
                t.waypoint.epsilon_rot = 0.4f;
                t.waypoint.speed = 8.0f;
                t.waypoint.speed_rot = 2.5f;

                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = true;
                pushTask(&r->taskmgr, t);
            }

            pushTask(&r->taskmgr, genTaskDelay(0.5));

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = false;
                pushTask(&r->taskmgr, t);
            }

            // Blue uses tag 7
            {
                Task t;
                t.type = TASK_ANGLE_TO_TAG_AUTO;
                t.photon_aligner.align_tag_id = 7;
                t.photon_aligner.angular_throttle_timer = 0.0;
                t.photon_aligner.shooter_align_epsilon = 0.2f;
                t.photon_aligner.delay_length = 0.0f;
                t.photon_aligner.timer_first = true;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAIT_FOR_FIRING_RPM;
                t.wait_rpm.rpm = 5300;
                t.wait_rpm.timer = 0.0;
                pushTask(&r->taskmgr, t);
            }
        
            {
                Task t;
                t.type = TASK_SEAT_RING;
                t.shooter.delay_timer = 0;
                t.shooter.delay_length = 0.1f;
                t.shooter.seat_speed_control = 1.0f;
                t.shooter.seat_speed_firing = 0;
                t.shooter.seat_first = false;
                pushTask(&r->taskmgr, t);
            }

            Pose prepickup;
            Pose pickup;
            for (int i = 0; i < 3; i++)
            {

                if (i == 0)
                {
                    prepickup = BLUE_NOTE_PREPICKUP_LEFT;
                    pickup = BLUE_NOTE_PICKUP_LEFT;
                }
                else if (i == 1)
                {
                    prepickup = BLUE_NOTE_PREPICKUP_MIDDLE;
                    pickup = BLUE_NOTE_PICKUP_MIDDLE;
                }
                else if (i == 2)
                {
                    prepickup = BLUE_NOTE_PREPICKUP_RIGHT;
                    pickup = BLUE_NOTE_PICKUP_RIGHT;
                }

                { // Sets pivot angle but doesn't wait for the move to finish
                    Task t;
                    t.type = TASK_SHOOTER_POSITIONING_NO_RETURN;
                    t.shooter.target_angle = 1.2f;
                    pushTask(&r->taskmgr, t);
                }     

                {
                    Task t;
                    t.type = TASK_WAYPOINT;
                    t.waypoint.target_pose = prepickup;
                    t.waypoint.epsilon = 1.0f;
                    t.waypoint.epsilon_rot = 0.7f;
                    t.waypoint.speed = 20.0f;
                    t.waypoint.speed_rot = 4.5f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_WAYPOINT;
                    t.waypoint.target_pose = prepickup;
                    t.waypoint.epsilon = 0.4f;
                    t.waypoint.epsilon_rot = 0.3f;
                    t.waypoint.speed = 8.0f;
                    t.waypoint.speed_rot = 3.0f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_SHOOTER_PULLER_START;
                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(0.5));

                {
                    Task t;
                    t.type = TASK_WAYPOINT_PULLER;
                    t.waypoint.target_pose = pickup;
                    t.waypoint.epsilon = 0.12f;
                    t.waypoint.epsilon_rot = 0.2f;
                    t.waypoint.speed = 10.0f;
                    t.waypoint.speed_rot = 1.5f;

                    pushTask(&r->taskmgr, t);
                }
                {
                    Task t;
                    t.type = TASK_AUTO_AWAIT_PULLER;
                    r->shooter.auto_await_timeout = 0.0;

                    pushTask(&r->taskmgr, t);
                }
                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = true;
                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(0.25));

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = false;
                    pushTask(&r->taskmgr, t);
                }
                

                {
                    Task t;
                    t.type = TASK_SHOOTER_PULLER_STOP;
                    pushTask(&r->taskmgr, t);
                }

                if (i == 2)
                {
                    {
                        Task t;
                        t.type = TASK_WAYPOINT;
                        t.waypoint.target_pose = BLUE_NOTE_PREPICKUP_MIDDLE;
                        t.waypoint.epsilon = 0.4f;
                        t.waypoint.epsilon_rot = 0.4f;
                        t.waypoint.speed = 8.0f;
                        t.waypoint.speed_rot = 2.5f;

                        pushTask(&r->taskmgr, t);
                    }

                    {
                        Task t;
                        t.type = TASK_MIDDLE_THE_WHEELS;
                        t.middle_wheels.enabled = true;
                        pushTask(&r->taskmgr, t);
                    }

                    pushTask(&r->taskmgr, genTaskDelay(0.4));

                    {
                        Task t;
                        t.type = TASK_MIDDLE_THE_WHEELS;
                        t.middle_wheels.enabled = false;
                        pushTask(&r->taskmgr, t);
                    }
                }

                // Blue uses tag 7
                {
                    Task t;
                    t.type = TASK_ANGLE_TO_TAG_AUTO;
                    t.photon_aligner.angular_throttle = 0.0;
                    t.photon_aligner.align_tag_id = 7;
                    t.photon_aligner.shooter_align_epsilon = 0.2f;
                    t.photon_aligner.delay_length = 0.5;
                    t.photon_aligner.timer_first = true;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_WAIT_FOR_FIRING_RPM;
                    t.wait_rpm.rpm = 5300;
                    t.wait_rpm.timer = 0.0;
                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(0.5));
            
                {
                    Task t;
                    t.type = TASK_SEAT_RING;
                    t.shooter.delay_timer = 0;
                    t.shooter.delay_length = 0.1f;
                    t.shooter.seat_speed_control = 1.0f;
                    t.shooter.seat_speed_firing = 0;
                    t.shooter.seat_first = false;
                    pushTask(&r->taskmgr, t);
                }

                if (i < 2)
                {
                    {
                        Task t;
                        t.type = TASK_WAYPOINT;
                        t.waypoint.target_pose = prepickup;
                        t.waypoint.epsilon = 0.6f;
                        t.waypoint.epsilon_rot = 0.3f;
                        t.waypoint.speed = 15.0f;
                        t.waypoint.speed_rot = 2.5f;

                        pushTask(&r->taskmgr, t);
                    }
                }

            }

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = true;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_SHOOTER_POSITIONING_NO_RETURN;
                t.shooter.target_angle = 0;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_SHOOTER_STOP;
                pushTask(&r->taskmgr, t);
            }
            
            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = true;
                pushTask(&r->taskmgr, t);
            }


        }

        case AUTO_TEST:
        {
            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.target_pose = BLUE_NOTE_PREPICKUP_LEFT;
                t.waypoint.epsilon = 0.2f;
                t.waypoint.epsilon_rot = 0.4f;
                t.waypoint.speed = 15.0f;
                t.waypoint.speed_rot = 3.5f;

                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = true;
                pushTask(&r->taskmgr, t);
            }

            pushTask(&r->taskmgr, genTaskDelay(1));

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = false;
                pushTask(&r->taskmgr, t);
            }

            pushTask(&r->taskmgr, genTaskDelay(1));

            Pose prepickup;
            Pose pickup;
            for (int i = 0; i < 3; i++)
            {

                if (i == 0)
                {
                    prepickup = BLUE_NOTE_PREPICKUP_LEFT;
                    pickup = BLUE_NOTE_PICKUP_LEFT;
                }
                else if (i == 1)
                {
                    prepickup = BLUE_NOTE_PREPICKUP_MIDDLE;
                    pickup = BLUE_NOTE_PICKUP_MIDDLE;
                }
                else if (i == 2)
                {
                    prepickup = BLUE_NOTE_PREPICKUP_RIGHT;
                    pickup = BLUE_NOTE_PICKUP_RIGHT;
                }
 

                {
                    Task t;
                    t.type = TASK_WAYPOINT;
                    t.waypoint.target_pose = prepickup;
                    t.waypoint.epsilon = 0.15f;
                    t.waypoint.epsilon_rot = 0.4f;
                    t.waypoint.speed = 10.0f;
                    t.waypoint.speed_rot = 1.5f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = true;
                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(1));

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = false;
                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(1));

                {
                    Task t;
                    t.type = TASK_WAYPOINT;
                    t.waypoint.target_pose = pickup;
                    t.waypoint.epsilon = 0.12f;
                    t.waypoint.epsilon_rot = 0.3f;
                    t.waypoint.speed = 10.0f;
                    t.waypoint.speed_rot = 1.5f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = true;
                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(1));

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = false;
                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(1));
                

            }

            

        }

        
    }

    return;
}


