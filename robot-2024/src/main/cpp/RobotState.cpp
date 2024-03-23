#include "RobotState.h"
#include <iostream>
#include "fennec/config.h"

void robotCmd(RobotData* r, RobotState state)
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
                t.type = TASK_SHOOTER_PULLER;
                pushTask(&r->taskmgr, t);
            }

            // pushTask(&r->taskmgr, genTaskDelay(0.5));

            {
                Task t;
                t.type = TASK_SEAT_RING;
                t.shooter.delay_timer = 0;
                t.shooter.delay_length = 0.05f;
                t.shooter.seat_speed_control = -0.8f;
                t.shooter.seat_speed_firing = 0.0f;
                pushTask(&r->taskmgr, t);
            }

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
                t.elevator.target_height = 0.275f;
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

        
    }   
    
}


void autoCmd(RobotData* r, AutoState state)
{
    switch(state)
    {
        case AUTO_BLUE_2_PIECE_AUTO:
        {
            frc::SmartDashboard::PutNumber("Auto init Delay", r->auto_init_delay);
            pushTask(&r->taskmgr, genTaskDelay(r->auto_init_delay));

            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                t.firing_motor.direction = 0;
                pushTask(&r->taskmgr, t);
            }

            // Blue uses tag 7
            {
                Task t;
                t.type = TASK_ANGLE_TO_TAG_AUTO;
                t.photon_aligner.align_tag_id = 7;
                t.photon_aligner.shooter_align_epsilon = 0.2f;
                t.photon_aligner.delay_length = 0.0f;
                t.photon_aligner.timer_first = true;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAIT_FOR_FIRING_RPM;
                t.wait_rpm.rpm = 4000;
                pushTask(&r->taskmgr, t);
            }
        
            {
                Task t;
                t.type = TASK_SEAT_RING;
                t.shooter.delay_timer = 0;
                t.shooter.delay_length = 0.1f;
                t.shooter.seat_speed_control = 0.8f;
                t.shooter.seat_speed_firing = 0;
                t.shooter.seat_first = false;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_SHOOTER_STOP;
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
                t.type = TASK_WAYPOINT;
                t.waypoint.target_pose = BLUE_NOTE_PREPICKUP_LEFT;
                t.waypoint.epsilon = 0.4f;
                t.waypoint.epsilon_rot = 0.4f;
                t.waypoint.speed = 8.0f;
                t.waypoint.speed_rot = 3.5f;

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


            {
                Task t;
                t.type = TASK_SHOOTER_PULLER_START;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAYPOINT;
                t.waypoint.target_pose = BLUE_NOTE_PICKUP_LEFT;
                t.waypoint.epsilon = 0.3f;
                t.waypoint.epsilon_rot = 0.2f;
                t.waypoint.speed = 8.0f;
                t.waypoint.speed_rot = 1.2f;

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

            pushTask(&r->taskmgr, genTaskDelay(0.5));

            {
                Task t;
                t.type = TASK_SHOOTER_PULLER_STOP;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_SEAT_RING;
                t.shooter.delay_timer = 0;
                t.shooter.delay_length = 0.05f;
                t.shooter.seat_speed_control = -0.8f;
                t.shooter.seat_speed_firing = 0.0f;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                t.firing_motor.direction = 0;
                pushTask(&r->taskmgr, t);
            }

            // Blue uses tag 7
            {
                Task t;
                t.type = TASK_ANGLE_TO_TAG_AUTO;
                t.photon_aligner.align_tag_id = 7;
                t.photon_aligner.shooter_align_epsilon = 0.2f;
                t.photon_aligner.delay_length = 0.5;
                t.photon_aligner.timer_first = true;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAIT_FOR_FIRING_RPM;
                t.wait_rpm.rpm = 5000;
                pushTask(&r->taskmgr, t);
            }
        
            {
                Task t;
                t.type = TASK_SEAT_RING;
                t.shooter.delay_timer = 0;
                t.shooter.delay_length = 0.1f;
                t.shooter.seat_speed_control = 0.8f;
                t.shooter.seat_speed_firing = 0;
                t.shooter.seat_first = false;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_SHOOTER_STOP;
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

        case AUTO_BLUE_4_PIECE:
        {
            frc::SmartDashboard::PutNumber("Auto init Delay", r->auto_init_delay);
            pushTask(&r->taskmgr, genTaskDelay(r->auto_init_delay));

            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                t.firing_motor.direction = 0;
                pushTask(&r->taskmgr, t);
            }

            // Blue uses tag 7
            {
                Task t;
                t.type = TASK_ANGLE_TO_TAG_AUTO;
                t.photon_aligner.align_tag_id = 7;
                t.photon_aligner.shooter_align_epsilon = 0.2f;
                t.photon_aligner.delay_length = 0.0f;
                t.photon_aligner.timer_first = true;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAIT_FOR_FIRING_RPM;
                t.wait_rpm.rpm = 5300;
                pushTask(&r->taskmgr, t);
            }
        
            {
                Task t;
                t.type = TASK_SEAT_RING;
                t.shooter.delay_timer = 0;
                t.shooter.delay_length = 0.1f;
                t.shooter.seat_speed_control = 0.8f;
                t.shooter.seat_speed_firing = 0;
                t.shooter.seat_first = false;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_SHOOTER_STOP;
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
                    t.waypoint.epsilon = 0.4f;
                    t.waypoint.epsilon_rot = 0.4f;
                    t.waypoint.speed = 8.0f;
                    t.waypoint.speed_rot = 3.5f;

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


                {
                    Task t;
                    t.type = TASK_SHOOTER_PULLER_START;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_WAYPOINT_PULLER;
                    t.waypoint.target_pose = pickup;
                    t.waypoint.epsilon = 0.3f;
                    t.waypoint.epsilon_rot = 0.2f;
                    t.waypoint.speed = 8.0f;
                    t.waypoint.speed_rot = 1.2f;

                    pushTask(&r->taskmgr, t);
                }
                {
                    Task t;
                    t.type = TASK_AUTO_AWAIT_PULLER;

                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(0.5));

                {
                    Task t;
                    t.type = TASK_SHOOTER_PULLER_STOP;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_SEAT_RING;
                    t.shooter.delay_timer = 0;
                    t.shooter.delay_length = 0.05f;
                    t.shooter.seat_speed_control = -0.8f;
                    t.shooter.seat_speed_firing = 0.0f;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_SHOOTER_FIRE;
                    t.firing_motor.direction = 0;
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
                    pushTask(&r->taskmgr, t);
                }
            
                {
                    Task t;
                    t.type = TASK_SEAT_RING;
                    t.shooter.delay_timer = 0;
                    t.shooter.delay_length = 0.1f;
                    t.shooter.seat_speed_control = 0.8f;
                    t.shooter.seat_speed_firing = 0;
                    t.shooter.seat_first = false;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_SHOOTER_STOP;
                    pushTask(&r->taskmgr, t);
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
            
        }

        case AUTO_RED_1_PIECE_AUTO:
        {
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
                t.waypoint.target_pose = {{14.95, 5.64}, M_PI / 2};
                t.waypoint.epsilon = 0.4f;
                t.waypoint.epsilon_rot = 0.2f;
                t.waypoint.speed = 6.0f;
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

            // Red uses tag 4
            {
                Task t;
                t.type = TASK_ANGLE_TO_TAG_AUTO;
                t.photon_aligner.align_tag_id = 4;
                t.photon_aligner.shooter_align_epsilon = 0.2f;
                t.photon_aligner.delay_length = 0.5;
                t.photon_aligner.timer_first = true;
                pushTask(&r->taskmgr, t);
            }


            {
                Task t;
                t.type = TASK_WAIT_FOR_FIRING_RPM;
                pushTask(&r->taskmgr, t);
            }
        
            {
                Task t;
                t.type = TASK_SEAT_RING;
                t.shooter.delay_timer = 0;
                t.shooter.delay_length = 0.1f;
                t.shooter.seat_speed_control = 0.8f;
                t.shooter.seat_speed_firing = 0;
                t.shooter.seat_first = false;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_SHOOTER_STOP;
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

        case AUTO_RED_4_PIECE:
        {
            frc::SmartDashboard::PutNumber("Auto init Delay", r->auto_init_delay);
            pushTask(&r->taskmgr, genTaskDelay(r->auto_init_delay));

            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                t.firing_motor.direction = 0;
                pushTask(&r->taskmgr, t);
            }

            // Red uses tag 4
            {
                Task t;
                t.type = TASK_ANGLE_TO_TAG_AUTO;
                t.photon_aligner.align_tag_id = 3;
                t.photon_aligner.shooter_align_epsilon = 0.2f;
                t.photon_aligner.delay_length = 0.0f;
                t.photon_aligner.timer_first = true;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_WAIT_FOR_FIRING_RPM;
                t.wait_rpm.rpm = 5300;
                pushTask(&r->taskmgr, t);
            }
        
            {
                Task t;
                t.type = TASK_SEAT_RING;
                t.shooter.delay_timer = 0;
                t.shooter.delay_length = 0.1f;
                t.shooter.seat_speed_control = 0.8f;
                t.shooter.seat_speed_firing = 0;
                t.shooter.seat_first = false;
                pushTask(&r->taskmgr, t);
            }

            {
                Task t;
                t.type = TASK_SHOOTER_STOP;
                pushTask(&r->taskmgr, t);
            }

            Pose prepickup;
            Pose pickup;
            for (int i = 0; i < 3; i++)
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
                    t.waypoint.epsilon = 0.4f;
                    t.waypoint.epsilon_rot = 0.4f;
                    t.waypoint.speed = 8.0f;
                    t.waypoint.speed_rot = 3.5f;

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


                {
                    Task t;
                    t.type = TASK_SHOOTER_PULLER_START;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_WAYPOINT_PULLER;
                    t.waypoint.target_pose = pickup;
                    t.waypoint.epsilon = 0.3f;
                    t.waypoint.epsilon_rot = 0.2f;
                    t.waypoint.speed = 8.0f;
                    t.waypoint.speed_rot = 1.2f;

                    pushTask(&r->taskmgr, t);
                }
                {
                    Task t;
                    t.type = TASK_AUTO_AWAIT_PULLER;

                    pushTask(&r->taskmgr, t);
                }

                pushTask(&r->taskmgr, genTaskDelay(0.5));

                {
                    Task t;
                    t.type = TASK_SHOOTER_PULLER_STOP;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_SEAT_RING;
                    t.shooter.delay_timer = 0;
                    t.shooter.delay_length = 0.05f;
                    t.shooter.seat_speed_control = -0.8f;
                    t.shooter.seat_speed_firing = 0.0f;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_SHOOTER_FIRE;
                    t.firing_motor.direction = 0;
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

                // Red uses tag 4
                {
                    Task t;
                    t.type = TASK_ANGLE_TO_TAG_AUTO;
                    t.photon_aligner.align_tag_id = 3;
                    t.photon_aligner.shooter_align_epsilon = 0.2f;
                    t.photon_aligner.delay_length = 0.5;
                    t.photon_aligner.timer_first = true;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_WAIT_FOR_FIRING_RPM;
                    t.wait_rpm.rpm = 5300;
                    pushTask(&r->taskmgr, t);
                }
            
                {
                    Task t;
                    t.type = TASK_SEAT_RING;
                    t.shooter.delay_timer = 0;
                    t.shooter.delay_length = 0.1f;
                    t.shooter.seat_speed_control = 0.8f;
                    t.shooter.seat_speed_firing = 0;
                    t.shooter.seat_first = false;
                    pushTask(&r->taskmgr, t);
                }

                {
                    Task t;
                    t.type = TASK_SHOOTER_STOP;
                    pushTask(&r->taskmgr, t);
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
            
        }

        case AUTO_TEST:
        {
            {
                Task t;
                t.type = TASK_SHOOTER_FIRE;
                t.firing_motor.direction = 0;
                pushTask(&r->taskmgr, t);
            }
        }
        
    }

    return;
}


