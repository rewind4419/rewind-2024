#include "RobotState.h"
#include <iostream>
#include "fennec/config.h"
#include "Robot.h"

#include <frc/shuffleboard/Shuffleboard.h>

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

bool first_prefire_shot = true;

bool note_left = true;
bool note_middle = true;
bool note_right = true;

extern nt::GenericEntry* singleWaypointSpeed;
extern nt::GenericEntry* singleWaypointSpeedRot;

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

        case SHOOTER_DELIVER_BLOOP:
        {
            printf("running bloop task\n");
            {
                Task t;
                t.type = TASK_SHOOTER_POSITIONING;
                t.shooter.target_angle = 0.9f; // - CFG_SHOOTER_ANGLE_OFFSET
                t.shooter.epsilon = 0.4f;
                pushTask(&r->taskmgr, t);
            }
            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                t.firing_motor.direction = 0.8f;
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
    }   
    
}


void autoCmd(RobotData* r, AutoState state, AutoAlliance alliance)
{
    switch(state)
    {
        case AUTO_4_PIECE:
        {
            Pose prepickup;
            Pose pickup;
            float april_tag;
            if(r->side == 0)
            {
                april_tag = 7;
                prepickup = BLUE_NOTE_PREPICKUP_RIGHT;
                
            }
            else if(r->side == 1)
            {
                april_tag = 4;
                prepickup = RED_NOTE_PREPICKUP_LEFT;
            }
            frc::SmartDashboard::PutNumber("Auto init Delay", r->auto_init_delay);
            pushTask(&r->taskmgr, genTaskDelay(r->auto_init_delay));

            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                t.firing_motor.direction = 0;
                pushTask(&r->taskmgr, t);
            }

            pushTask(&r->taskmgr, genTaskDelay(1.1));


            if(r->photon.n_tags == 0)
            {
                {
                    Task t;
                    t.type = TASK_DRIVETRAIN_VELOCITY;
                    t.drivetrain_velocity.target_angular_velocity = 0;
                    t.drivetrain_velocity.target_velocity = {0, 10};
                    t.drivetrain_velocity.timer = 0;
                    t.drivetrain_velocity.length = 0.5;
                    pushTask(&r->taskmgr, t);

                }

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = true;
                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(0.2));

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = false;
                    pushTask(&r->taskmgr, t);
                }
            }

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                t.waypoint.target_pose = prepickup;
                t.waypoint.epsilon = 0.4f;
                t.waypoint.epsilon_rot = 0.4f;
                t.waypoint.speed = 12.0f;
                t.waypoint.speed_rot = 2.0f;

                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                t.waypoint.target_pose = prepickup;
                t.waypoint.epsilon = 0.4f;
                t.waypoint.epsilon_rot = 0.25f;
                t.waypoint.speed = 7.0f;
                t.waypoint.speed_rot = 1.0f;

                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = true;
                pushTask(&r->taskmgr, t);
            }

            pushTask(&r->taskmgr, genTaskDelay(0.2));

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
                t.photon_aligner.align_tag_id = april_tag;
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

            for (int i = 0; i < 3; i++)
            {
                
                if(r->side == 0)
                {
                    if (i == 0)
                    {
                        prepickup = BLUE_NOTE_PREPICKUP_RIGHT;
                        pickup = BLUE_NOTE_PICKUP_RIGHT;
                    }
                    else if (i == 1)
                    {
                        prepickup = BLUE_NOTE_PREPICKUP_MIDDLE;
                        pickup = BLUE_NOTE_PICKUP_MIDDLE;
                    }
                    else if (i == 2)
                    {
                        prepickup = BLUE_NOTE_PREPICKUP_LEFT;
                        pickup = BLUE_NOTE_PICKUP_LEFT;
                    }
                }
                else if(r->side == 1)
                {
                    if (i == 0)
                    {
                        prepickup = RED_NOTE_PREPICKUP_LEFT;
                        pickup = RED_NOTE_PICKUP_LEFT;
                    }
                    else if (i == 1)
                    {
                        prepickup = RED_NOTE_PREPICKUP_MIDDLE;
                        pickup = RED_NOTE_PICKUP_MIDDLE;
                    }
                    else if (i == 2)
                    {
                        prepickup = RED_NOTE_PREPICKUP_RIGHT;
                        pickup = RED_NOTE_PICKUP_RIGHT;
                    }
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
                t.waypoint.tag_aligner = false;
                    t.waypoint.target_pose = prepickup;
                    t.waypoint.epsilon = 0.5f;
                    t.waypoint.epsilon_rot = 0.3f;
                    t.waypoint.speed = 12.0f;
                    t.waypoint.speed_rot = 2.0f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                    t.waypoint.target_pose = prepickup;
                    t.waypoint.epsilon = 0.3f;
                    t.waypoint.epsilon_rot = 0.25f;
                    t.waypoint.speed = 7.0f;
                    t.waypoint.speed_rot = 1.0f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_SHOOTER_PULLER_START;
                    pushTask(&r->taskmgr, t);
                }

                // {
                //     Task t;
                //     t.type = TASK_MIDDLE_THE_WHEELS;
                //     t.middle_wheels.enabled = true;
                //     pushTask(&r->taskmgr, t);
                // }

                // pushTask(&r->taskmgr, genTaskDelay(0.1));

                // {
                //     Task t;
                //     t.type = TASK_MIDDLE_THE_WHEELS;
                //     t.middle_wheels.enabled = false;
                //     pushTask(&r->taskmgr, t);
                // }

                {
                    Task t;
                    t.type = TASK_WAYPOINT_PULLER;
                    t.waypoint.target_pose = pickup;
                    t.waypoint.epsilon = 0.2f;
                    t.waypoint.epsilon_rot = 0.25f;
                    t.waypoint.speed = 7.0f;
                    t.waypoint.speed_rot = 1.0f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = true;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_AUTO_AWAIT_PULLER;
                    pushTask(&r->taskmgr, t);
                }

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
                t.waypoint.tag_aligner = false;
                        t.waypoint.target_pose = prepickup;
                        t.waypoint.epsilon = 0.5f;
                        t.waypoint.epsilon_rot = 0.25f;
                        t.waypoint.speed = 12.0f;
                        t.waypoint.speed_rot = 1.3f;

                        pushTask(&r->taskmgr, t);
                    }

                    {
                        Task t;
                        t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                        t.waypoint.target_pose = prepickup;
                        t.waypoint.epsilon = 0.2f;
                        t.waypoint.epsilon_rot = 0.1f;
                        t.waypoint.speed = 6.0f;
                        t.waypoint.speed_rot = 1.3f;

                        pushTask(&r->taskmgr, t);
                    }

                    {
                        Task t;
                        t.type = TASK_MIDDLE_THE_WHEELS;
                        t.middle_wheels.enabled = true;
                        pushTask(&r->taskmgr, t);
                    }

                }

                // Blue uses tag 7
                {
                    Task t;
                    t.type = TASK_ANGLE_TO_TAG_AUTO;
                    t.photon_aligner.angular_throttle = 0.0;
                    t.photon_aligner.align_tag_id = april_tag;
                    t.photon_aligner.shooter_align_epsilon = 0.2f;
                    t.photon_aligner.delay_length = 0.5f;
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

                // pushTask(&r->taskmgr, genTaskDelay(0.5));
            
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
                        t.waypoint.tag_aligner = false;
                        t.waypoint.target_pose = prepickup;
                        t.waypoint.epsilon = 0.5f;
                        t.waypoint.epsilon_rot = 0.3f;
                        t.waypoint.speed = 10.0f;
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

            // {
            //     Task t;
            //     t.type = TASK_SHOOTER_STOP;
            //     pushTask(&r->taskmgr, t);
            // }
            
            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = true;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_DRIVETRAIN_OVERRIDE;
                pushTask(&r->taskmgr, t);
            }
        }

        case AUTO_TEST:
        {
            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                t.waypoint.target_pose = BLUE_NOTE_PREPICKUP_LEFT;
                t.waypoint.epsilon = 0.1f;
                t.waypoint.epsilon_rot = 0.1f;
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
                t.waypoint.tag_aligner = false;
                    t.waypoint.target_pose = prepickup;
                    t.waypoint.epsilon = 0.1f;
                    t.waypoint.epsilon_rot = 0.1f;
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
                t.waypoint.tag_aligner = false;
                    t.waypoint.target_pose = pickup;
                    t.waypoint.epsilon = 0.1f;
                    t.waypoint.epsilon_rot = 0.1f;
                    t.waypoint.speed = 4.0f;
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

        case AUTO_SINGLE_TEST:
        {
            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                t.waypoint.target_pose = BLUE_NOTE_PREPICKUP_LEFT;
                t.waypoint.epsilon = 0.0f;
                t.waypoint.epsilon_rot = 0.0f;
                t.waypoint.speed = singleWaypointSpeed->GetDouble(1.0);
                t.waypoint.speed_rot = singleWaypointSpeedRot->GetDouble(1.0);

                pushTask(&r->taskmgr, t);
            }
        }

        case AUTO_SHOOT_WHILE_INTAKING:
        {
            Pose prepickup;
            Pose pickup;
            float april_tag;
            if(r->side == 0)
            {
                april_tag = 7;
                prepickup = BLUE_NOTE_PREPICKUP_RIGHT;
                pickup = BLUE_NOTE_PICKUP_RIGHT;
                
            }
            else if(r->side == 1)
            {
                april_tag = 4;
                prepickup = RED_NOTE_PREPICKUP_LEFT;
                pickup = RED_NOTE_PICKUP_LEFT;
            }

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
                t.type = TASK_AUTO_AIM_ACTIVATION;
                t.auto_aim.activated = true;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                t.waypoint.target_pose = prepickup;
                t.waypoint.epsilon = 0.4f;
                t.waypoint.epsilon_rot = 0.4f;
                t.waypoint.speed = 12.0f;
                t.waypoint.speed_rot = 2.0f;

                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                t.waypoint.target_pose = prepickup;
                t.waypoint.epsilon = 0.15f;
                t.waypoint.epsilon_rot = 0.25f;
                t.waypoint.speed = 7.0f;
                t.waypoint.speed_rot = 1.0f;

                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = true;
                pushTask(&r->taskmgr, t);
            }

            pushTask(&r->taskmgr, genTaskDelay(0.1));

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = false;
                pushTask(&r->taskmgr, t);
            }

            // pushTask(&r->taskmgr, genTaskDelay(1));

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = true;
                t.waypoint.target_pose = pickup;
                t.waypoint.epsilon = 0.4f;
                t.waypoint.epsilon_rot = 0.4f;
                t.waypoint.speed = 5.0f;
                t.waypoint.speed_rot = 2.5f;

                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = true;
                t.waypoint.target_pose = pickup;
                t.waypoint.epsilon = 0.2f;
                t.waypoint.epsilon_rot = 0.2f;
                t.waypoint.speed = 4.5f;
                t.waypoint.speed_rot = 2.5f;

                pushTask(&r->taskmgr, t);
            }

            pushTask(&r->taskmgr, genTaskDelay(0.5));

            {
                Task t;
                t.type = TASK_AUTO_AIM_ACTIVATION;
                t.auto_aim.activated = false;
                pushTask(&r->taskmgr, t);
            }

            { // Sets pivot angle but doesn't wait for the move to finish
                Task t;
                t.type = TASK_SHOOTER_POSITIONING_NO_RETURN;
                t.shooter.target_angle = 0;
                pushTask(&r->taskmgr, t);
            } 

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                t.waypoint.target_pose = RED_NOTE_PRE_STAGE;
                t.waypoint.epsilon = 0.3f;
                t.waypoint.epsilon_rot = 0.2f;
                t.waypoint.speed = 10;
                t.waypoint.speed_rot = 2.5;

                pushTask(&r->taskmgr, t);
            }

            // {
            //     Task t;
            //     t.type = TASK_WAYPOINT;
            //     t.waypoint.tag_aligner = false;
            //     t.waypoint.target_pose = RED_NOTE_UNDER_STAGE;
            //     t.waypoint.epsilon = 0.2f;
            //     t.waypoint.epsilon_rot = 0.2f;
            //     t.waypoint.speed = 6.0f;
            //     t.waypoint.speed_rot = 2;

            //     pushTask(&r->taskmgr, t);
            // }

            {
                Task t;
                t.type = TASK_DRIVETRAIN_VELOCITY;
                t.drivetrain_velocity.target_angular_velocity = 0;
                t.drivetrain_velocity.target_velocity = {0, 10};
                t.drivetrain_velocity.timer = 0;
                t.drivetrain_velocity.length = 1;
                pushTask(&r->taskmgr, t);

            }

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = true;
                pushTask(&r->taskmgr, t);
            }

            pushTask(&r->taskmgr, genTaskDelay(0.1));

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = false;
                pushTask(&r->taskmgr, t);
            }

            { // Sets pivot angle but doesn't wait for the move to finish
                Task t;
                t.type = TASK_SHOOTER_POSITIONING_NO_RETURN;
                t.shooter.target_angle = 1.2f;
                pushTask(&r->taskmgr, t);
            } 

            {
                Task t;
                t.type = TASK_SHOOTER_PULLER_START;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAYPOINT_PULLER;
                t.waypoint.target_pose = RED_NOTE_MIDDLE_3;
                t.waypoint.epsilon = 0.2f;
                t.waypoint.epsilon_rot = 0.25f;
                t.waypoint.speed = 7.0f;
                t.waypoint.speed_rot = 1.0f;

                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = true;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_AUTO_AWAIT_PULLER;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = false;
                pushTask(&r->taskmgr, t);
            }

            { // Sets pivot angle but doesn't wait for the move to finish
                Task t;
                t.type = TASK_SHOOTER_POSITIONING_NO_RETURN;
                t.shooter.target_angle = 0;
                pushTask(&r->taskmgr, t);
            } 
            
            {
                Task t;
                t.type = TASK_SHOOTER_PULLER_STOP;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                t.waypoint.target_pose = RED_NOTE_POST_STAGE;
                t.waypoint.epsilon = 0.2f;
                t.waypoint.epsilon_rot = 0.2f;
                t.waypoint.speed = 10.0f;
                t.waypoint.speed_rot = 2;

                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_DRIVETRAIN_VELOCITY;
                t.drivetrain_velocity.target_angular_velocity = 0;
                t.drivetrain_velocity.target_velocity = {0, -10};
                t.drivetrain_velocity.timer = 0;
                t.drivetrain_velocity.length = 1;
                pushTask(&r->taskmgr, t);

            }

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = true;
                pushTask(&r->taskmgr, t);
            }

            pushTask(&r->taskmgr, genTaskDelay(0.1));

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
                t.photon_aligner.align_tag_id = april_tag;
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

        }

        case AUTO_5_PIECE:
        {
            Pose prepickup;
            Pose pickup;
            float april_tag;
            if(r->side == 0)
            {
                april_tag = 7;
                prepickup = BLUE_NOTE_PREPICKUP_RIGHT;
                pickup = BLUE_NOTE_PICKUP_RIGHT;
                
            }
            else if(r->side == 1)
            {
                april_tag = 4;
                prepickup = RED_NOTE_PREPICKUP_LEFT;
                pickup = RED_NOTE_PICKUP_LEFT;

            }
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
                t.type = TASK_AUTO_AIM_ACTIVATION;
                t.auto_aim.activated = true;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                t.waypoint.target_pose = prepickup;
                t.waypoint.epsilon = 0.4f;
                t.waypoint.epsilon_rot = 0.4f;
                t.waypoint.speed = 12.0f;
                t.waypoint.speed_rot = 2.0f;

                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                t.waypoint.target_pose = prepickup;
                t.waypoint.epsilon = 0.15f;
                t.waypoint.epsilon_rot = 0.25f;
                t.waypoint.speed = 7.0f;
                t.waypoint.speed_rot = 1.0f;

                pushTask(&r->taskmgr, t);
            }

            // {
            //     Task t;
            //     t.type = TASK_MIDDLE_THE_WHEELS;
            //     t.middle_wheels.enabled = true;
            //     pushTask(&r->taskmgr, t);
            // }

            // pushTask(&r->taskmgr, genTaskDelay(0.1));

            // {
            //     Task t;
            //     t.type = TASK_MIDDLE_THE_WHEELS;
            //     t.middle_wheels.enabled = false;
            //     pushTask(&r->taskmgr, t);
            // }

            // pushTask(&r->taskmgr, genTaskDelay(1));

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = true;
                t.waypoint.target_pose = pickup;
                t.waypoint.epsilon = 0.4f;
                t.waypoint.epsilon_rot = 0.4f;
                t.waypoint.speed = 5.0f;
                t.waypoint.speed_rot = 2.5f;

                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = true;
                t.waypoint.target_pose = pickup;
                t.waypoint.epsilon = 0.2f;
                t.waypoint.epsilon_rot = 0.2f;
                t.waypoint.speed = 4.5f;
                t.waypoint.speed_rot = 2.5f;

                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_PHOTON_AIM_TIMER_RESET;
                pushTask(&r->taskmgr, t);
            }

            // pushTask(&r->taskmgr, genTaskDelay(0.6));



            // {
            //     Task t;
            //     t.type = TASK_AUTO_AIM_ACTIVATION;
            //     t.auto_aim.activated = false;
            //     pushTask(&r->taskmgr, t);
            // }

            // { // Sets pivot angle but doesn't wait for the move to finish
            //     Task t;
            //     t.type = TASK_SHOOTER_POSITIONING_NO_RETURN;
            //     t.shooter.target_angle = 0;
            //     pushTask(&r->taskmgr, t);
            // } 

            for (int i = 0; i < 3; i++)
            {
                
                if(r->side == 0)
                {
                    if (i == 0)
                    {
                        continue;
                        prepickup = BLUE_NOTE_PREPICKUP_RIGHT;
                        pickup = BLUE_NOTE_PICKUP_RIGHT;
                    }
                    else if (i == 1)
                    {
                        prepickup = BLUE_NOTE_PREPICKUP_MIDDLE;
                        pickup = BLUE_NOTE_PICKUP_MIDDLE;
                    }
                    else if (i == 2)
                    {
                        prepickup = BLUE_NOTE_PREPICKUP_LEFT;
                        pickup = BLUE_NOTE_PICKUP_LEFT;
                    }
                }
                else if(r->side == 1)
                {
                    if (i == 0)
                    {
                        continue;
                        prepickup = RED_NOTE_PREPICKUP_LEFT;
                        pickup = RED_NOTE_PICKUP_LEFT;
                    }
                    else if (i == 1)
                    {
                        prepickup = RED_NOTE_PREPICKUP_MIDDLE;
                        pickup = RED_NOTE_PICKUP_MIDDLE;
                    }
                    else if (i == 2)
                    {
                        prepickup = RED_NOTE_PREPICKUP_RIGHT;
                        pickup = RED_NOTE_PICKUP_RIGHT;
                    }
                }

                Pose pre_pre_pickup = prepickup;
                pre_pre_pickup.position.y += 0.3;

                {
                    Task t;
                    t.type = TASK_WAYPOINT;
                    t.waypoint.tag_aligner = false;
                    t.waypoint.target_pose = pre_pre_pickup;
                    t.waypoint.epsilon = 0.5f;
                    t.waypoint.epsilon_rot = 0.3f;
                    t.waypoint.speed = 12.0f;
                    t.waypoint.speed_rot = 2.0f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = true;
                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(0.05));

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = false;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_WAYPOINT;
                    t.waypoint.tag_aligner = false;
                    t.waypoint.target_pose = prepickup;
                    t.waypoint.epsilon = 0.2f;
                    t.waypoint.epsilon_rot = 0.25f;
                    t.waypoint.speed = 7.0f;
                    t.waypoint.speed_rot = 1.0f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_SHOOTER_PULLER_START;
                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(0.1));

                {
                    Task t;
                    t.type = TASK_WAYPOINT;
                    t.waypoint.tag_aligner = true;
                    t.waypoint.target_pose = pickup;
                    t.waypoint.epsilon = 0.2f;
                    t.waypoint.epsilon_rot = 0.25f;
                    t.waypoint.speed = 7.0f;
                    t.waypoint.speed_rot = 2.0f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_PHOTON_AIM_TIMER_RESET;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = true;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_AUTO_AWAIT_PULLER;
                    pushTask(&r->taskmgr, t);
                }

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
                        t.waypoint.tag_aligner = false;
                        t.waypoint.target_pose = prepickup;
                        t.waypoint.epsilon = 0.5f;
                        t.waypoint.epsilon_rot = 0.25f;
                        t.waypoint.speed = 12.0f;
                        t.waypoint.speed_rot = 1.3f;

                        pushTask(&r->taskmgr, t);
                    }

                    {
                        Task t;
                        t.type = TASK_WAYPOINT;
                        t.waypoint.tag_aligner = false;
                        t.waypoint.target_pose = prepickup;
                        t.waypoint.epsilon = 0.2f;
                        t.waypoint.epsilon_rot = 0.1f;
                        t.waypoint.speed = 6.0f;
                        t.waypoint.speed_rot = 1.3f;

                        pushTask(&r->taskmgr, t);
                    }

                    {
                        Task t;
                        t.type = TASK_MIDDLE_THE_WHEELS;
                        t.middle_wheels.enabled = true;
                        pushTask(&r->taskmgr, t);
                    }

                }

                // Blue uses tag 7
                // {
                //     Task t;
                //     t.type = TASK_ANGLE_TO_TAG_AUTO;
                //     t.photon_aligner.angular_throttle = 0.0;
                //     t.photon_aligner.align_tag_id = april_tag;
                //     t.photon_aligner.shooter_align_epsilon = 0.2f;
                //     t.photon_aligner.delay_length = 0.5f;
                //     t.photon_aligner.timer_first = true;
                //     pushTask(&r->taskmgr, t);
                // }

                // {
                //     Task t;
                //     t.type = TASK_WAIT_FOR_FIRING_RPM;
                //     t.wait_rpm.rpm = 5300;
                //     t.wait_rpm.timer = 0.0;
                //     pushTask(&r->taskmgr, t);
                // }

                // pushTask(&r->taskmgr, genTaskDelay(0.5));
            
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
                        t.waypoint.tag_aligner = false;
                        t.waypoint.target_pose = prepickup;
                        t.waypoint.epsilon = 0.5f;
                        t.waypoint.epsilon_rot = 0.3f;
                        t.waypoint.speed = 11.0f;
                        t.waypoint.speed_rot = 2.5f;

                        pushTask(&r->taskmgr, t);
                    }
                }
            }

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = false;
                pushTask(&r->taskmgr, t);
            }

            { // Sets pivot angle but doesn't wait for the move to finish
                Task t;
                t.type = TASK_SHOOTER_POSITIONING_NO_RETURN;
                t.shooter.target_angle = 0;
                pushTask(&r->taskmgr, t);
            }

            Pose pre_prestage = RED_NOTE_PRE_STAGE;
            pre_prestage.position.x += 0.3;
            // pre_prestage.position.y -= 0.3;

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                t.waypoint.target_pose = pre_prestage;
                t.waypoint.epsilon = 0.3f;
                t.waypoint.epsilon_rot = 0.2f;
                t.waypoint.speed = 6.5;
                t.waypoint.speed_rot = 2.5;

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
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                t.waypoint.target_pose = RED_NOTE_PRE_STAGE;
                t.waypoint.epsilon = 0.2f;
                t.waypoint.epsilon_rot = 0.15f;
                t.waypoint.speed = 10;
                t.waypoint.speed_rot = 1.5;

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

            pushTask(&r->taskmgr, genTaskDelay(0.1));

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = false;
                pushTask(&r->taskmgr, t);
            }


            // {
            //     Task t;
            //     t.type = TASK_WAYPOINT;
            //     t.waypoint.tag_aligner = false;
            //     t.waypoint.target_pose = RED_NOTE_UNDER_STAGE;
            //     t.waypoint.epsilon = 0.2f;
            //     t.waypoint.epsilon_rot = 0.2f;
            //     t.waypoint.speed = 6.0f;
            //     t.waypoint.speed_rot = 2;

            //     pushTask(&r->taskmgr, t);
            // }

            {
                Task t;
                t.type = TASK_DRIVETRAIN_VELOCITY;
                t.drivetrain_velocity.target_angular_velocity = 0;
                t.drivetrain_velocity.target_velocity = {-2, 10};
                t.drivetrain_velocity.timer = 0;
                t.drivetrain_velocity.length = 1;
                pushTask(&r->taskmgr, t);

            }

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = true;
                pushTask(&r->taskmgr, t);
            }

            pushTask(&r->taskmgr, genTaskDelay(0.1));

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = false;
                pushTask(&r->taskmgr, t);
            }

            { // Sets pivot angle but doesn't wait for the move to finish
                Task t;
                t.type = TASK_SHOOTER_POSITIONING_NO_RETURN;
                t.shooter.target_angle = 1.2f;
                pushTask(&r->taskmgr, t);
            } 

            {
                Task t;
                t.type = TASK_SHOOTER_PULLER_START;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAYPOINT_PULLER;
                t.waypoint.target_pose = RED_NOTE_MIDDLE_3;
                t.waypoint.epsilon = 0.2f;
                t.waypoint.epsilon_rot = 0.25f;
                t.waypoint.speed = 7.0f;
                t.waypoint.speed_rot = 1.0f;

                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = true;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_AUTO_AWAIT_PULLER;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = false;
                pushTask(&r->taskmgr, t);
            }

            { // Sets pivot angle but doesn't wait for the move to finish
                Task t;
                t.type = TASK_SHOOTER_POSITIONING_NO_RETURN;
                t.shooter.target_angle = 0;
                pushTask(&r->taskmgr, t);
            } 
            
            {
                Task t;
                t.type = TASK_SHOOTER_PULLER_STOP;
                pushTask(&r->taskmgr, t);
            }
            
            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = true;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_DRIVETRAIN_OVERRIDE;
                pushTask(&r->taskmgr, t);
            }
        }

        case AUTO_PREFIRE_LEAVE_COMMUNITY_RIGHT:
        {

            Pose prepickup;
            Pose pickup;
            float april_tag;
            if(r->side == 0)
            {
                april_tag = 7;
                prepickup = BLUE_NOTE_PREPICKUP_RIGHT;
                
            }
            else if(r->side == 1)
            {
                april_tag = 4;
                prepickup = RED_NOTE_PREPICKUP_LEFT;

            }
            frc::SmartDashboard::PutNumber("Auto init Delay", r->auto_init_delay);
            pushTask(&r->taskmgr, genTaskDelay(r->auto_init_delay));

            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                t.firing_motor.direction = 0;
                pushTask(&r->taskmgr, t);
            }

            if(r->photon.n_tags == 0)
            {
                {
                    Task t;
                    t.type = TASK_DRIVETRAIN_VELOCITY;
                    t.drivetrain_velocity.target_angular_velocity = 0;
                    t.drivetrain_velocity.target_velocity = {0, 10};
                    t.drivetrain_velocity.timer = 0;
                    t.drivetrain_velocity.length = 0.5;
                    pushTask(&r->taskmgr, t);

                }

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = true;
                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(0.2));

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = false;
                    pushTask(&r->taskmgr, t);
                }


            }

            {
                Task t;
                t.type = TASK_DRIVETRAIN_VELOCITY;
                t.drivetrain_velocity.target_angular_velocity = 0;
                t.drivetrain_velocity.target_velocity = {0.81506 * 10,  0.57 * 10};
                t.drivetrain_velocity.timer = 0;
                t.drivetrain_velocity.length = 1.5;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_DRIVETRAIN_OVERRIDE;
                pushTask(&r->taskmgr, t);
            }

        }

        case AUTO_PREFIRE_LEAVE_COMMUNITY_LEFT:
        {

            Pose prepickup;
            Pose pickup;
            Pose center_pose_1;
            Pose center_pose_2;

            float delay_vel_1;
            float delay_vel_2;
            // Center field x = 8.270620346069336
            //Center field y = 5.547867774963379

            float april_tag;
            if(r->side == 0)
            {
                april_tag = 7;
                prepickup = BLUE_NOTE_PREPICKUP_RIGHT;
                delay_vel_1 = 1.0;
                delay_vel_2 = 1.0;
                // center_pose_1 = {}
                
            }
            else if(r->side == 1)
            {
                april_tag = 4;
                prepickup = RED_NOTE_PREPICKUP_LEFT;
                delay_vel_1 = 0.5;
                delay_vel_2 = 1.5;

            }
            frc::SmartDashboard::PutNumber("Auto init Delay", r->auto_init_delay);
            pushTask(&r->taskmgr, genTaskDelay(r->auto_init_delay));

            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                t.firing_motor.direction = 0;
                pushTask(&r->taskmgr, t);
            }

            if(r->photon.n_tags == 0)
            {
                {
                    Task t;
                    t.type = TASK_DRIVETRAIN_VELOCITY;
                    t.drivetrain_velocity.target_angular_velocity = 0;
                    t.drivetrain_velocity.target_velocity = {0, 10};
                    t.drivetrain_velocity.timer = 0;
                    t.drivetrain_velocity.length = delay_vel_1;
                    pushTask(&r->taskmgr, t);

                }

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = true;
                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(0.2));

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = false;
                    pushTask(&r->taskmgr, t);
                }

            }

            {
                Task t;
                t.type = TASK_DRIVETRAIN_VELOCITY;
                t.drivetrain_velocity.target_angular_velocity = 0;
                t.drivetrain_velocity.target_velocity = {- 0.81506 * 10,  0.57 * 10};
                t.drivetrain_velocity.timer = 0;
                t.drivetrain_velocity.length = 1.5;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                t.waypoint.target_pose = prepickup;
                t.waypoint.epsilon = 0.4f;
                t.waypoint.epsilon_rot = 0.4f;
                t.waypoint.speed = 12.0f;
                t.waypoint.speed_rot = 2.0f;

                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                t.waypoint.target_pose = prepickup;
                t.waypoint.epsilon = 0.4f;
                t.waypoint.epsilon_rot = 0.25f;
                t.waypoint.speed = 7.0f;
                t.waypoint.speed_rot = 1.0f;

                pushTask(&r->taskmgr, t);
            }



            {
                Task t;
                t.type = TASK_DRIVETRAIN_OVERRIDE;
                pushTask(&r->taskmgr, t);
            }

        }

        case AUTO_4_PIECE_CONFIG:
        {
            Pose prepickup;
            Pose pickup;
            float april_tag;
            if(r->side == 0)
            {
                april_tag = 7;
                prepickup = BLUE_NOTE_PREPICKUP_RIGHT;
                
            }
            else if(r->side == 1)
            {
                april_tag = 4;
                prepickup = RED_NOTE_PREPICKUP_LEFT;

            }
            frc::SmartDashboard::PutNumber("Auto init Delay", r->auto_init_delay);
            pushTask(&r->taskmgr, genTaskDelay(r->auto_init_delay));

            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                t.firing_motor.direction = 0;
                pushTask(&r->taskmgr, t);
            }

            if(r->photon.n_tags == 0)
            {
                {
                    Task t;
                    t.type = TASK_DRIVETRAIN_VELOCITY;
                    t.drivetrain_velocity.target_angular_velocity = 0;
                    t.drivetrain_velocity.target_velocity = {0, 10};
                    t.drivetrain_velocity.timer = 0;
                    t.drivetrain_velocity.length = 0.5;
                    pushTask(&r->taskmgr, t);

                }

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = true;
                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(0.2));

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = false;
                    pushTask(&r->taskmgr, t);
                }
            }

            if(first_prefire_shot)
            {
                {
                    Task t;
                    t.type = TASK_WAYPOINT;
                    t.waypoint.tag_aligner = false;
                    t.waypoint.target_pose = prepickup;
                    t.waypoint.epsilon = 0.4f;
                    t.waypoint.epsilon_rot = 0.4f;
                    t.waypoint.speed = 12.0f;
                    t.waypoint.speed_rot = 2.0f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_WAYPOINT;
                    t.waypoint.tag_aligner = false;
                    t.waypoint.target_pose = prepickup;
                    t.waypoint.epsilon = 0.4f;
                    t.waypoint.epsilon_rot = 0.25f;
                    t.waypoint.speed = 7.0f;
                    t.waypoint.speed_rot = 1.0f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = true;
                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(0.2));

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
                    t.photon_aligner.align_tag_id = april_tag;
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
            }
            
            for (int i = 0; i < 3; i++)
            {
                
                // Blue side
                if(r->side == 0)
                {
                    if (i == 0)
                    {
                        if(!note_right) continue;
                        prepickup = BLUE_NOTE_PREPICKUP_RIGHT;
                        pickup = BLUE_NOTE_PICKUP_RIGHT;
                    }
                    else if (i == 1)
                    {
                        if(!note_middle) continue;
                        prepickup = BLUE_NOTE_PREPICKUP_MIDDLE;
                        pickup = BLUE_NOTE_PICKUP_MIDDLE;
                    }
                    else if (i == 2)
                    {
                        if(!note_left) continue;
                        prepickup = BLUE_NOTE_PREPICKUP_LEFT;
                        pickup = BLUE_NOTE_PICKUP_LEFT;
                    }
                }
                else if(r->side == 1)
                {
                    if (i == 0)
                    {
                        if(!note_left) continue;
                        prepickup = RED_NOTE_PREPICKUP_LEFT;
                        pickup = RED_NOTE_PICKUP_LEFT;
                    }
                    else if (i == 1)
                    {
                        if(!note_middle) continue;
                        prepickup = RED_NOTE_PREPICKUP_MIDDLE;
                        pickup = RED_NOTE_PICKUP_MIDDLE;
                    }
                    else if (i == 2)
                    {
                        if(!note_right) continue;
                        prepickup = RED_NOTE_PREPICKUP_RIGHT;
                        pickup = RED_NOTE_PICKUP_RIGHT;
                    }
                }




                { // Sets pivot angle but doesn't wait for the move to finish
                    Task t;
                    t.type = TASK_SHOOTER_POSITIONING_NO_RETURN;
                    t.shooter.target_angle = 1.2f;
                    pushTask(&r->taskmgr, t);
                }  

                printf("inside loop");   

                {
                    Task t;
                    t.type = TASK_WAYPOINT;
                    t.waypoint.tag_aligner = false;
                    t.waypoint.target_pose = prepickup;
                    t.waypoint.epsilon = 0.5f;
                    t.waypoint.epsilon_rot = 0.3f;
                    t.waypoint.speed = 12.0f;
                    t.waypoint.speed_rot = 2.0f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                    t.waypoint.target_pose = prepickup;
                    t.waypoint.epsilon = 0.3f;
                    t.waypoint.epsilon_rot = 0.25f;
                    t.waypoint.speed = 7.0f;
                    t.waypoint.speed_rot = 1.0f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_SHOOTER_PULLER_START;
                    pushTask(&r->taskmgr, t);
                }

                // {
                //     Task t;
                //     t.type = TASK_MIDDLE_THE_WHEELS;
                //     t.middle_wheels.enabled = true;
                //     pushTask(&r->taskmgr, t);
                // }

                // pushTask(&r->taskmgr, genTaskDelay(0.1));

                // {
                //     Task t;
                //     t.type = TASK_MIDDLE_THE_WHEELS;
                //     t.middle_wheels.enabled = false;
                //     pushTask(&r->taskmgr, t);
                // }

                {
                    Task t;
                    t.type = TASK_WAYPOINT_PULLER;
                    t.waypoint.target_pose = pickup;
                    t.waypoint.epsilon = 0.2f;
                    t.waypoint.epsilon_rot = 0.25f;
                    t.waypoint.speed = 7.0f;
                    t.waypoint.speed_rot = 1.0f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = true;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_AUTO_AWAIT_PULLER;
                    pushTask(&r->taskmgr, t);
                }

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
                    t.waypoint.tag_aligner = false;
                        t.waypoint.target_pose = prepickup;
                        t.waypoint.epsilon = 0.5f;
                        t.waypoint.epsilon_rot = 0.25f;
                        t.waypoint.speed = 12.0f;
                        t.waypoint.speed_rot = 1.3f;

                        pushTask(&r->taskmgr, t);
                    }

                    {
                        Task t;
                        t.type = TASK_WAYPOINT;
                        t.waypoint.tag_aligner = false;
                        t.waypoint.target_pose = prepickup;
                        t.waypoint.epsilon = 0.2f;
                        t.waypoint.epsilon_rot = 0.1f;
                        t.waypoint.speed = 6.0f;
                        t.waypoint.speed_rot = 1.3f;

                        pushTask(&r->taskmgr, t);
                    }

                    {
                        Task t;
                        t.type = TASK_MIDDLE_THE_WHEELS;
                        t.middle_wheels.enabled = true;
                        pushTask(&r->taskmgr, t);
                    }

                }

                // Blue uses tag 7
                {
                    Task t;
                    t.type = TASK_ANGLE_TO_TAG_AUTO;
                    t.photon_aligner.angular_throttle = 0.0;
                    t.photon_aligner.align_tag_id = april_tag;
                    t.photon_aligner.shooter_align_epsilon = 0.2f;
                    t.photon_aligner.delay_length = 0.5f;
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

                // pushTask(&r->taskmgr, genTaskDelay(0.5));
            
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
                        t.waypoint.tag_aligner = false;
                        t.waypoint.target_pose = prepickup;
                        t.waypoint.epsilon = 0.5f;
                        t.waypoint.epsilon_rot = 0.3f;
                        t.waypoint.speed = 10.0f;
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

            // {
            //     Task t;
            //     t.type = TASK_SHOOTER_STOP;
            //     pushTask(&r->taskmgr, t);
            // }
            
            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = true;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_DRIVETRAIN_OVERRIDE;
                pushTask(&r->taskmgr, t);
            }
        }
        
        case AUTO_PREFIRE_LEAVE_COMMUNITY:
        {

            Pose prepickup;
            Pose pickup;
            float april_tag;
            if(r->side == 0)
            {
                april_tag = 7;
                prepickup = BLUE_NOTE_PREPICKUP_RIGHT;
                
            }
            else if(r->side == 1)
            {
                april_tag = 4;
                prepickup = RED_NOTE_PREPICKUP_LEFT;

            }
            frc::SmartDashboard::PutNumber("Auto init Delay", r->auto_init_delay);
            pushTask(&r->taskmgr, genTaskDelay(r->auto_init_delay));

            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                t.firing_motor.direction = 0;
                pushTask(&r->taskmgr, t);
            }

            // if(r->photon.n_tags == 0)
            // {
                {
                    Task t;
                    t.type = TASK_DRIVETRAIN_VELOCITY;
                    t.drivetrain_velocity.target_angular_velocity = 0;
                    t.drivetrain_velocity.target_velocity = {0, 10};
                    t.drivetrain_velocity.timer = 0;
                    t.drivetrain_velocity.length = 0.5;
                    pushTask(&r->taskmgr, t);

                }

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = true;
                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(0.2));

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = false;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_ANGLE_TO_TAG_AUTO;
                    t.photon_aligner.align_tag_id = april_tag;
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


            // }

            {
                Task t;
                t.type = TASK_DRIVETRAIN_OVERRIDE;
                pushTask(&r->taskmgr, t);
            }

        }
        case AUTO_4_PIECE_REVERSED:
        {
            Pose prepickup;
            Pose pickup;
            float april_tag;
            if(r->side == 0)
            {
                april_tag = 7;
                prepickup = BLUE_NOTE_PREPICKUP_LEFT;
                
            }
            else if(r->side == 1)
            {
                april_tag = 4;
                prepickup = RED_NOTE_PREPICKUP_RIGHT;
            }
            frc::SmartDashboard::PutNumber("Auto init Delay", r->auto_init_delay);
            pushTask(&r->taskmgr, genTaskDelay(r->auto_init_delay));

            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                t.firing_motor.direction = 0;
                pushTask(&r->taskmgr, t);
            }

            if(r->photon.n_tags == 0)
            {
                {
                    Task t;
                    t.type = TASK_DRIVETRAIN_VELOCITY;
                    t.drivetrain_velocity.target_angular_velocity = 0;
                    t.drivetrain_velocity.target_velocity = {0, 10};
                    t.drivetrain_velocity.timer = 0;
                    t.drivetrain_velocity.length = 0.5;
                    pushTask(&r->taskmgr, t);

                }

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = true;
                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(0.2));

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = false;
                    pushTask(&r->taskmgr, t);
                }
            }

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                t.waypoint.target_pose = prepickup;
                t.waypoint.epsilon = 0.4f;
                t.waypoint.epsilon_rot = 0.4f;
                t.waypoint.speed = 12.0f;
                t.waypoint.speed_rot = 2.0f;

                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                t.waypoint.target_pose = prepickup;
                t.waypoint.epsilon = 0.4f;
                t.waypoint.epsilon_rot = 0.25f;
                t.waypoint.speed = 7.0f;
                t.waypoint.speed_rot = 1.0f;

                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = true;
                pushTask(&r->taskmgr, t);
            }

            pushTask(&r->taskmgr, genTaskDelay(0.2));

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
                t.photon_aligner.align_tag_id = april_tag;
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

            for (int i = 0; i < 3; i++)
            {
                
                if(r->side == 0)
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
                }
                else if(r->side == 1)
                {
                    if (i == 0)
                    {

                        prepickup = RED_NOTE_PREPICKUP_RIGHT;
                        pickup = RED_NOTE_PICKUP_RIGHT;

                    }
                    else if (i == 1)
                    {
                        prepickup = RED_NOTE_PREPICKUP_MIDDLE;
                        pickup = RED_NOTE_PICKUP_MIDDLE;
                    }
                    else if (i == 2)
                    {

                        prepickup = RED_NOTE_PREPICKUP_LEFT;
                        pickup = RED_NOTE_PICKUP_LEFT;

                    }
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
                t.waypoint.tag_aligner = false;
                    t.waypoint.target_pose = prepickup;
                    t.waypoint.epsilon = 0.5f;
                    t.waypoint.epsilon_rot = 0.3f;
                    t.waypoint.speed = 12.0f;
                    t.waypoint.speed_rot = 2.0f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_WAYPOINT;
                t.waypoint.tag_aligner = false;
                    t.waypoint.target_pose = prepickup;
                    t.waypoint.epsilon = 0.3f;
                    t.waypoint.epsilon_rot = 0.25f;
                    t.waypoint.speed = 7.0f;
                    t.waypoint.speed_rot = 1.0f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_SHOOTER_PULLER_START;
                    pushTask(&r->taskmgr, t);
                }

                // {
                //     Task t;
                //     t.type = TASK_MIDDLE_THE_WHEELS;
                //     t.middle_wheels.enabled = true;
                //     pushTask(&r->taskmgr, t);
                // }

                // pushTask(&r->taskmgr, genTaskDelay(0.1));

                // {
                //     Task t;
                //     t.type = TASK_MIDDLE_THE_WHEELS;
                //     t.middle_wheels.enabled = false;
                //     pushTask(&r->taskmgr, t);
                // }

                {
                    Task t;
                    t.type = TASK_WAYPOINT_PULLER;
                    t.waypoint.target_pose = pickup;
                    t.waypoint.epsilon = 0.2f;
                    t.waypoint.epsilon_rot = 0.25f;
                    t.waypoint.speed = 7.0f;
                    t.waypoint.speed_rot = 1.0f;

                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_MIDDLE_THE_WHEELS;
                    t.middle_wheels.enabled = true;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_AUTO_AWAIT_PULLER;
                    pushTask(&r->taskmgr, t);
                }

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

                // if (i < 2)
                {
                    {
                        Task t;
                        t.type = TASK_WAYPOINT;
                        t.waypoint.tag_aligner = false;
                        t.waypoint.target_pose = prepickup;
                        t.waypoint.epsilon = 0.5f;
                        t.waypoint.epsilon_rot = 0.25f;
                        t.waypoint.speed = 12.0f;
                        t.waypoint.speed_rot = 1.3f;

                        pushTask(&r->taskmgr, t);
                    }

                    {
                        Task t;
                        t.type = TASK_WAYPOINT;
                        t.waypoint.tag_aligner = false;
                        t.waypoint.target_pose = prepickup;
                        t.waypoint.epsilon = 0.2f;
                        t.waypoint.epsilon_rot = 0.1f;
                        t.waypoint.speed = 6.0f;
                        t.waypoint.speed_rot = 1.3f;

                        pushTask(&r->taskmgr, t);
                    }

                    {
                        Task t;
                        t.type = TASK_MIDDLE_THE_WHEELS;
                        t.middle_wheels.enabled = true;
                        pushTask(&r->taskmgr, t);
                    }

                }

            
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
                    t.photon_aligner.angular_throttle = 0.0;
                    t.photon_aligner.align_tag_id = april_tag;
                    t.photon_aligner.shooter_align_epsilon = 0.2f;
                    t.photon_aligner.delay_length = 0.5f;
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

                // pushTask(&r->taskmgr, genTaskDelay(0.5));
            
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

                // if (i < 2)
                // {
                //     {
                //         Task t;
                //         t.type = TASK_WAYPOINT;
                //         t.waypoint.tag_aligner = false;
                //         t.waypoint.target_pose = prepickup;
                //         t.waypoint.epsilon = 0.5f;
                //         t.waypoint.epsilon_rot = 0.3f;
                //         t.waypoint.speed = 10.0f;
                //         t.waypoint.speed_rot = 2.5f;

                //         pushTask(&r->taskmgr, t);
                //     }
                // }

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

            // {
            //     Task t;
            //     t.type = TASK_SHOOTER_STOP;
            //     pushTask(&r->taskmgr, t);
            // }
            
            {
                Task t;
                t.type = TASK_MIDDLE_THE_WHEELS;
                t.middle_wheels.enabled = true;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_DRIVETRAIN_OVERRIDE;
                pushTask(&r->taskmgr, t);
            }
        }
    }

    return;
}


