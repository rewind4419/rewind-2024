// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"


#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/DataLogManager.h>

nt::GenericEntry* firingVelocity;
nt::GenericEntry* firingReadyIndicator;
nt::GenericEntry* currentAutoTask;
nt::GenericEntry* currentDriverState;
nt::GenericEntry* kP;
nt::GenericEntry* kI;
nt::GenericEntry* kD;



nt::GenericEntry* autoMode;

nt::GenericEntry* localizerX;
nt::GenericEntry* localizerY;
nt::GenericEntry* localizerR;

nt::GenericEntry* waypointTaskEpsilon;
nt::GenericEntry* waypointTaskRotEpsilon;

nt::GenericEntry* aprilTagDist;
nt::GenericEntry* aprilTagAngle;

nt::GenericEntry* ampScoreHeight;
nt::GenericEntry* ampScoreAngle;



void initRobot(RobotData *r, RobotMode mode)
{
//How to use smart dashboard
// frc::SmartDashboard::PutData("Field", &r.field);
// frc::SmartDashboard::PutNumber("AutoMode", 0);

    // initialize the sensors
    printf("Initializing Robot");

    initDrivetrain(&r->drivetrain);
    initDrivetrainController(&r->drivetrain_controller);
    initIntake(&r->intake);
    initShooter(&r->shooter);
    initElevator(&r->elevator);

    r->taskmgr = TaskMgr();
    r->sensor_imu = new AHRS(frc::SPI::Port::kMXP);
    r->integrated_imu_pos = {};
    r->imu_basis = 0;
    r->sensor_imu->ZeroYaw();

    // r->lastCalledState = STATE_NONE;
    r->robotState.robotMode = MODE_DEFAULT;

    // // Shuffleboard
    firingVelocity = frc::Shuffleboard::GetTab("Main").Add("Firing Velocity", 0.0).GetEntry();
    firingReadyIndicator = frc::Shuffleboard::GetTab("Main").Add("Firing Ready", false).GetEntry();
    currentAutoTask = frc::Shuffleboard::GetTab("State").Add("Currnt Auto Task ID", 0).GetEntry();
    currentDriverState = frc::Shuffleboard::GetTab("State").Add("Current Driver State ID", 0).GetEntry();
    kP = frc::Shuffleboard::GetTab("Shooter").Add("kP", 0.4).GetEntry();
    kI = frc::Shuffleboard::GetTab("Shooter").Add("kI", 0.0).GetEntry();
    kD = frc::Shuffleboard::GetTab("Shooter").Add("kI", 0.0).GetEntry();

    

    autoMode = frc::Shuffleboard::GetTab("Main").Add("AUTO MODE", 0).GetEntry();

    localizerX = frc::Shuffleboard::GetTab("Localizer").Add("localizerX", 0.0).GetEntry();
    localizerY = frc::Shuffleboard::GetTab("Localizer").Add("localizerY", 0.0).GetEntry();
    localizerR = frc::Shuffleboard::GetTab("Localizer").Add("localizerR", 0.0).GetEntry();

    waypointTaskEpsilon = frc::Shuffleboard::GetTab("Localizer").Add("waypoint epsilon", -1.0).GetEntry();
    waypointTaskRotEpsilon = frc::Shuffleboard::GetTab("Localizer").Add("waypoint rot epsilon", -1.0).GetEntry();

    ampScoreHeight = frc::Shuffleboard::GetTab("Main").Add("Amp Score Height", 0.275).GetEntry();
    ampScoreAngle = frc::Shuffleboard::GetTab("Main").Add("Amp Socre Angle", 1.45).GetEntry();

}

void robotModeInit(RobotData *r, RobotMode new_mode)
{
    r->shooter.axis_motors[0]->SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    r->shooter.axis_motors[1]->SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);

    frc::DataLogManager::Stop();

    frc::DataLogManager::Start();

    r->robotState.robotMode = MODE_DEFAULT;
    
    r->taskmgr = TaskMgr{};

    resetShooter(&r->shooter);
    resetElevator(&r->elevator);

    if(new_mode == ROBOT_AUTO)
    {
        AutoAlliance autoAlliance = A_ALLIANCE_NONE;
        if(r->side == 0)
        {
            autoAlliance = A_ALLIANCE_BLUE;
        }
        else if(r->side == 1)
        {
            autoAlliance = A_ALLIANCE_RED;
        }

        if (autoAlliance == A_ALLIANCE_NONE)
        {
            printf("WARNING: AUTO ALLIANCE WAS NONE, ABORTING AUTO!!\n");
        }
        else
        {
            // if (autoMode->GetInteger(0) == 0)
            //     printf("WARNING: AUTO MODE = 0, RUNNING NO AUTO\n");

            autoCmd(r, AUTO_4_PIECE, autoAlliance);
        }
    }

    if(new_mode == ROBOT_TELEOP)
    {
    }

    // Change to dependent on case later
    r->enable_time = 0;
    r->localiser = {};

    r->sensor_imu->ZeroYaw();
    r->aligner = ALGN_NONE;
    r->drivetrain_controller.mode = DRIVECTRL_VELOCITY;
    r->drivetrain_controller.ctrl.velocity.velocity = {0, 0};
    r->drivetrain_controller.ctrl.velocity.angular_velocity = 0;

    r->imu_basis = 0;

}

void updateRobot(RobotData *r, float time_step, RobotMode mode)
{
    r->auto_init_delay = frc::SmartDashboard::GetNumber("Auto Initial Delay", 0);

    // calibrateShooterAngle(&r->shooter);
    // printCalibrationData(&r->drivetrain);
    // calibrateElevator(&r->elevator);
    // r->side = frc::SmartDashboard::GetNumber("Init Side", 0);

    localizerX->SetDouble(r->localiser.pose_estimate.position.x);
    localizerY->SetDouble(r->localiser.pose_estimate.position.y);
    localizerR->SetDouble(r->localiser.pose_estimate.rotation);

    r->driverstation_side = frc::DriverStation::GetAlliance();

    if(r->driverstation_side == frc::DriverStation::kRed) r->side = 1;
    else if(r->driverstation_side == frc::DriverStation::kBlue) r->side = 0;

    //Red = 1; Blue = 0;
    frc::SmartDashboard::PutNumber("Side", r->side);

    if (mode == ROBOT_DISABLE) return;

    r->enable_time +=r->delta_time;
    r->delta_time = time_step;

    updateGamepad(&r->input);


    r->latest_odometry_frame = getDrivetrainOdometry(&r->drivetrain);

    // reset the target velocities
    if (r->drivetrain_controller.mode == DRIVECTRL_VELOCITY)
    {
        r->drivetrain_controller.ctrl.velocity.velocity = v2{0, 0};
        r->drivetrain_controller.ctrl.velocity.angular_velocity = 0;
    }
    else if (r->drivetrain_controller.mode == DRIVECTRL_THROTTLE)
    {
        r->drivetrain_controller.ctrl.throttle.throttle = v2{0, 0};
        r->drivetrain_controller.ctrl.throttle.angular_throttle = 0;
    }

    if(mode == ROBOT_DISABLE) return;

    if (mode == ROBOT_TELEOP)
    {
        {
            // Drivetrain
            v2 input_translation = r->input.driver.joystick_left;

            if (length(input_translation) > 1)
                input_translation = normalize(input_translation);

            // Speed of the bot based on gamepad bumpers
            float driver_speed_target = CFG_DRIVER_SPEED_NORMAL;
            float driver_speed_target_rotation = CFG_DRIVER_SPEED_NORMAL_ROT;

            // Make input curve Square
            // input_translation = input_translation * v2{length(input_translation), length(input_translation)};

            if ( r->input.driver.bumper_right.held)
            {
                driver_speed_target = CFG_DRIVER_SPEED_SURGERY;
                driver_speed_target_rotation = CFG_DRIVER_SPEED_SURGERY_ROT;
            }

            else if ( r->input.driver.bumper_left.held)
            {
                driver_speed_target = CFG_DRIVER_SPEED_SPRINT;
                driver_speed_target_rotation = CFG_DRIVER_SPEED_SPRINT_ROT;
            }

            //Slowly amp up speed
            const float SPEED_CHANGE_RATE = 1;
            
            {
                float speed_diff = driver_speed_target - r->driver_speed;

                speed_diff = CLAMP(speed_diff, -SPEED_CHANGE_RATE * r->delta_time, SPEED_CHANGE_RATE * r->delta_time);

                r->driver_speed += speed_diff;
            }

            float curr_speed = r->driver_speed;
            float curr_speed_rot = driver_speed_target_rotation;


            // Resets the rotation of the drivetrain
            if ( r->input.driver.y.held)
            {
                r->imu_basis = degToRad( r->sensor_imu->GetYaw());
            }

            if ( r->input.driver.x.held)
            {
                r->imu_basis = degToRad( r->sensor_imu->GetYaw()) + M_PI / 2.0f;
            }

            if ( r->input.driver.b.held)
            {
                r->imu_basis = degToRad( r->sensor_imu->GetYaw()) - M_PI / 2.0f;
            }

            if ( r->input.driver.a.held)
            {
                r->imu_basis = degToRad( r->sensor_imu->GetYaw()) + M_PI;
            }

            float imu_yaw = degToRad( r->sensor_imu->GetYaw()) - r->imu_basis;
            input_translation = rotate(input_translation, -imu_yaw);

            if ( r->input.driver.x.held || r->input.driver.y.held || r->input.driver.b.held || r->input.driver.a.held) r->held_rotation = imu_yaw;
        
            input_translation = input_translation * curr_speed;

            r->global_input_translation = input_translation;
            
            if ( r->input.driver.big_button.held)
            {
                r->middle_wheels = true;
            }


            float imu_rotation_radians = imu_yaw;

            v2 robot_dir = v2{sinf(imu_rotation_radians), cosf(imu_rotation_radians)};



            //Fixing wrong sided rotaton -Matteo
            v2 driver_joystick_right = r->input.driver.joystick_right;
            driver_joystick_right.x *= -1;
            driver_joystick_right.y *= -1;


            //USE THIS FOR APRIL TAG ALIGNER CODE IF THE OTHER ISNT EFFICIENT

            float angle_diff = acosf(dot(robot_dir, normalize( driver_joystick_right)));
            v2 robot_right = rotate(robot_dir, M_PI / 2);
            if (dot(robot_right, normalize( driver_joystick_right)) < 0)
            {
                angle_diff = -angle_diff;
            }

            // deadzone
            // if(length( driver_joystick_right) < 0.5f)
            {
                angle_diff = 0;
            }

            float angle01 = angle_diff / M_PI;
            // const float reactiveness = 0.5; // higher reactivity = closer to 0, straight reactiveness curve = 1
            float power_curve = pow(fabsf(angle01), CFG_DRIVER_ABSOLUTE_ROTATION_REACTIVENESS);

            power_curve = power_curve * sign(angle01);

            float adjustment_rotation = driver_joystick_right.x * CFG_DRIVER_ADJUSTMENT_ROTATION_SENSITIVITY * (curr_speed_rot / CFG_DRIVER_SPEED_NORMAL);

            if (length(input_translation) > 0.15 || fabsf(power_curve + adjustment_rotation) > 0.15)
            {
                r->middle_wheels = false;
            }

            if (fabsf(power_curve + adjustment_rotation) > 0.15)
            {
                r->aligner = ALGN_NONE;
            }

            if ( r->middle_wheels)
            {
                v2 targets[DrivetrainSwerve_Count];
                targets[DrivetrainSwerve_BL] = v2{1, 1};
                targets[DrivetrainSwerve_BR] = v2{-1, 1};
                targets[DrivetrainSwerve_FL] = v2{1, -1};
                targets[DrivetrainSwerve_FR] = v2{-1, -1};
                drivetrainUpdateRawVectors(&r->drivetrain, targets, r->delta_time, true);
            }
            else
            {
            if ( r->aligner != ALGN_NONE)
            {
                float auto_rotater = 0;

                v2 robot_dir = v2{sinf(imu_rotation_radians), cosf(imu_rotation_radians)};

                v2 align_dir = v2{0, 1};

                switch ( r->aligner)
                {
                case ALGN_FORWARD:
                    align_dir = v2{0, 1};
                    break;
                case ALGN_BACKWARD:
                    align_dir = v2{0, -1};
                    break;
                default:
                    break;
                }

                float my_angle_diff = angleBetween(robot_dir, align_dir);

                float my_angle01 = my_angle_diff / M_PI;

                auto_rotater = evalPid(&r->aligner_pid, my_angle01, r->delta_time);

                // printf("Rotater: %f\n", my_power_curve);

                // drivetrainUpdate(&r.drivetrain, input_translation, auto_rotater, r->delta_time);
                r->drivetrain_controller.mode = DRIVECTRL_THROTTLE;
                r->drivetrain_controller.ctrl.throttle.throttle = input_translation;
                r->drivetrain_controller.ctrl.throttle.angular_throttle = auto_rotater;
                }
                else
                {
                    // the idea is that it holds rotation

                    // Sherwin: This is kind of a hack to get rid of hold rotation, uncomment the below line to add it back
                    // (Hold rotation = whenever the driver isn't touching the controller, the PID is maintaining a certain rotation)
                    // (Im pretty sure reason our robot randomly rotates is because it was trying to maintain its rotation while the IMU was disconnected)
                    //if (fabsf(power_curve + adjustment_rotation) > 0.015)
                    {
                        r->held_rotation = imu_yaw;
                    }

                    v2 robot_facing = rotate(v2{0, 1}, imu_yaw);
                    v2 hold_facing = rotate(v2{0, 1}, r->held_rotation);

                    // this is zero if you're actively turning the robot, bc held_rotation is getting updated
                    float hold_angle_diff = angleBetween(robot_facing, hold_facing);

                    float hold_angle01 = hold_angle_diff / M_PI;
                    
                    r->drivetrain_controller.mode = DRIVECTRL_THROTTLE;
                    r->drivetrain_controller.ctrl.throttle.throttle = input_translation;
                    r->drivetrain_controller.ctrl.throttle.angular_throttle = power_curve + adjustment_rotation + evalPid(&r->holder_pid, -1 * hold_angle01, r->delta_time);

                    // printf("real angle throttle = %f\n", power_curve + adjustment_rotation + evalPid(&r->holder_pid, -1 * hold_angle01, r->delta_time));

                    // drivetrainUpdate(&r->drivetrain, input_translation, power_curve + adjustment_rotation + evalPid(&r->holder_pid, hold_angle01, r->delta_time), r->delta_time);
                }
            }
        }


        auto *in = &r->input;

        // MATE CODE MATE CODE MATE CODE MATE CODE MATE CODE MATE CODE
        // (technically its SUBSYTEM CODE)
        // (there are some driver controls here too)

        // in->mate = in->driver;

        if (in->mate.big_button.down)
        {
            r->taskmgr = TaskMgr();
            printf("Attempted to clear queue\n");
        }

        if (in->mate.b.held)
        {
            r->robotState.robotMode = MODE_DEFAULT;
        }

        frc::SmartDashboard::PutBoolean("Beam Break", r->shooter.beam_break.Get());


        r->shooter.shooter_pid.kP = kP->GetDouble(0.4);
        r->shooter.shooter_pid.kI = kI->GetDouble(0.0);
        r->shooter.shooter_pid.kD = kD->GetDouble(0.0);
        
        currentDriverState->SetInteger(r->robotState.robotMode);
        switch (r->robotState.robotMode)
        {
        case MODE_DEFAULT:
            // Always down in default mode
            r->elevator.target_height = 0;
            r->shooter.target_angle = 0;

            r->shooter.firing_mode = true;
            r->ready_fire_amp = false;
            r->shooter.firing_motor_speed = -0.75;

            //r->shooter.control_motor_speed = (in->mate.trigger_right * 0.5 + 0.5);
            if (r->shooter.beam_break.Get())
            {
                r->shooter.control_motor_speed = ((in->mate.trigger_right * 0.5 + 0.5)- (in->mate.trigger_left * 0.5 + 0.5)) * CFG_CONTROL_PULLER_MAX_SPEED;
                r->intake.intake_speed = ((in->mate.trigger_right * 0.5 + 0.5)- (in->mate.trigger_left * 0.5 + 0.5)) * CFG_INTAKE_PULLER_MAX_SPEED;
            }
            else
            {
                r->shooter.control_motor_speed = - (in->mate.trigger_left * 0.5 + 0.5);
                r->intake.intake_speed = -(in->mate.trigger_left * 0.5 + 0.5);
            }

            if (in->mate.a.down)
            {
                r->robotState.robotMode = MODE_INTAKING;
            }

            if (in->mate.y.down)
            {
                r->robotState.robotMode = MODE_AMP;
            }

            if (in->mate.x.down)
            {
                r->robotState.robotMode = MODE_SHOOTING;
                r->shooter.target_angle = 0.9f - CFG_SHOOTER_ANGLE_OFFSET;
                r->shooter.shooter_pid.errorAccum = 0;
            }
            if (in->mate.share_button.down)
            {
                r->robotState.robotMode = MODE_CLIMBING;
                r->shooter.target_angle = 1.2f;
                r->elevator.target_height = 0.225; //Might need to change?
            }
            break;
        case MODE_INTAKING:
            if (r->shooter.beam_break.Get() == false)
            {
                // note detected, finish the intake
                r->robotState.robotMode = MODE_DEFAULT;
                printf("STOPPING INTAKE DUE TO BEAM BREAK\n");
                r->shooter.control_motor_speed = 0;
                r->intake.intake_speed = 0;
            }

            r->shooter.target_angle = 1.2f;
            r->elevator.target_height = 0.0;

            r->shooter.firing_mode = true;
            r->ready_fire_amp = false;
            r->shooter.firing_motor_speed = -0.5;

            r->intake.intake_speed = CFG_INTAKE_MAX_SPEED;
            r->shooter.control_motor_speed = CFG_CONTROL_PULLER_MAX_SPEED;

            frc::SmartDashboard::PutNumber("Intake gamepad input", (in->mate.trigger_right * 0.5 + 0.5));

            if (in->mate.a.up)
            {
                r->robotState.robotMode = MODE_DEFAULT;
            }
            break;
        case MODE_AMP:
            // r->shooter.target_angle = ampScoreAngle->GetDouble(1.45);
            // r->elevator.target_height = ampScoreHeight->GetDouble(0.275);

            r->shooter.target_angle = (1.45);
            r->elevator.target_height = (0.31);

            r->shooter.firing_mode = true;
            r->ready_fire_amp = false;
            r->shooter.firing_motor_speed = 0.5;

             r->shooter.control_motor_speed = (in->mate.trigger_right * 0.5 + 0.5) - (in->mate.trigger_left * 0.5 + 0.5);
            r->intake.intake_speed = 0;
            
            break;
        case MODE_SHOOTING:

            r->shooter.firing_mode = true;
            r->ready_fire_amp = false;
            r->shooter.firing_motor_speed = -1.0;

            if ((in->driver.trigger_right * 0.5 + 0.5) > 0.5)
            {
                r->shooter.control_motor_speed = 1.0f;
            }
            else
            {
                r->shooter.control_motor_speed = (in->mate.trigger_right * 0.5 + 0.5);
            }
            r->intake.intake_speed = 0;

            if (in->mate.bumper_left.held || in->driver.trigger_left > 0.5)
            {
                if(r->side == 0) calculateVision(7, r);
                if(r->side == 1) calculateVision(4, r);
            }

            if (in->mate.a.down)
            {
                r->robotState.robotMode = MODE_INTAKING;
            }
            if (in->mate.y.down)
            {
                r->robotState.robotMode = MODE_AMP;
            }
            break;
        case MODE_CLIMBING:

            r->elevator.target_height = CLAMP(r-elevator.target_height + in->mate.joystick_left.y * 0.05, 0, CFG_ELEVATOR_RANGE);

            r->shooter.firing_mode = false;
            r->ready_fire_amp = false;
            
            r->shooter.control_motor_speed = 0;
            r->intake.intake_speed = 0;

            if (in->mate.option_button.down)
            {
                r->shooter.target_angle = 1.2f;
                r->elevator.target_height = 0.0;
            }
            if (in->mate.share_button.down)
            {
                r->shooter.target_angle = 1.2f;
                r->elevator.target_height = 0.225;
            }

            break;
        }

        if (r->robotState.robotMode == MODE_SHOOTING)
        {
            r->shooter.shooter_pid.kP = 2.0;
        }
        else
        {
            r->shooter.shooter_pid.kP = 0.4;
        }

        float shooterVelocity = r->shooter.firing_encoder->GetVelocity();
        firingVelocity->SetDouble(shooterVelocity);
        if (fabsf(shooterVelocity) > 5400.0f)
        {
            firingReadyIndicator->SetBoolean(true);
        }
        else
        {
            firingReadyIndicator->SetBoolean(false);
        }

        // // Case that robot is in firing mode
        // if(r->shooter.firing_mode && !r->shooter.intake_task && !r->ready_fire_amp)
        // {
        //     // During Firing Mode Left Trigger
        //     if(in->mate.trigger_left > 0.01f && r->photon.first_aim)
        //     {
        //         robotCmd(r, ANGLE_TO_SPEAKER);
        //     }
        //     //During Firing Mode Using Right Trigger
        //     if (in->mate.trigger_right > 0.01f)
        //     {
        //         r->intake.intake_speed = in->mate.trigger_right / 3;
        //         r->shooter.control_motor_speed = in->mate.trigger_right;

        //     }
        //     //During Firing Mode, Nothing
        //     else
        //     {
        //         r->intake.intake_speed = 0;
        //         r->shooter.control_motor_speed = 0;
        //     }
        // }
        // // Case that robot is in intake mode
        // else if(!r->shooter.intake_task && r->shooter.beam_break.Get() == true)
        // {
        //     //Not During Firing Mode, Right Trigger
        //     if (in->mate.trigger_right > 0.01f)
        //     {
        //         r->intake.intake_speed = in->mate.trigger_right / 2;
        //         r->shooter.control_motor_speed = in->mate.trigger_right;
        //     }

        //     //Not During Firing Mode, Left Trigger
        //     else if(in->mate.trigger_left > 0.01f)
        //     {
        //         r->intake.intake_speed = -in->mate.trigger_left / 2;
        //         r->shooter.control_motor_speed = -in->mate.trigger_left / 2;

        //     }
        //     //Not During Firing Mode, Nothing
        //     else
        //     {
        //         r->intake.intake_speed = 0;
        //         r->shooter.control_motor_speed = 0;
        //     } 
        // }
        
        // //Return to Rest Position
        // if(in->mate.b.down)
        // {
        //     r->ready_fire_amp = false;
        //     {
        //         Task t;
        //         t.type = TASK_ELEVATOR_POSITIONING;
        //         t.elevator.target_height = 0;
        //         t.shooter.epsilon = 0.2f;
        //         pushTask(&r->taskmgr, t);
        //     }   
            

        //     {
        //         Task t;
        //         t.type = TASK_SHOOTER_POSITIONING;
        //         t.shooter.target_angle = 0;
        //         t.shooter.epsilon = 0.4f;
        //         pushTask(&r->taskmgr, t);
        //     }     
        // }

        // if(in->mate.trigger_right > 0.01 && r->ready_fire_amp)
        // {
        //     r->shooter.amp_mode = true;
        // }
        // else
        // {
        //     r->shooter.amp_mode = false;
        // }

        // if(in->mate.x.down)
        // {
        //     robotCmd(r, SHOOTER_DELIVER_SPEAKER);
        // }

        // if(in->mate.y.down)
        // {
        //     robotCmd(r, SHOOTER_DELIVER_AMP);
        // }

        // if(in->mate.a.down)
        // {
        //     r->ready_fire_amp = false;
        //     robotCmd(r, INTAKE_TRANSFER);
        // }

        // if(in->mate.share_button.down) //Need to Change Button - Nethra
        // {
        //     printf("CLIMB POSITIONING\n");
        //     robotCmd(r, CLIMB_POSITIONING);
        // }

        // if(in->mate.option_button.down) //Need to Change Button - Nethra
        // {
        //     printf("CLIMB\n");
        //     robotCmd(r, CLIMBING);
        // }
      
        // // Press Right Trigger And firing motor task is on, 3rd is just to make sure it only queues once
        // if (in->mate.trigger_right > 0.01 && r->shooter.firing_motor_task == true && r->shooter.shooter_first_time == true) 
        // {
        //     robotCmd(r, SHOOTER_STOP);
        //     r->shooter.shooter_first_time = false;
        // }
        // // Press Right Bumper And firing motor task is on, 3rd is just to make sure it only queues once
        // else if(in->mate.bumper_right.down && r->shooter.firing_motor_task == true && r->shooter.shooter_first_time == true) 
        // {
        //     {
        //         Task t;
        //         t.type = TASK_SHOOTER_STOP;
        //         pushTask(&r->taskmgr, t);
        //     }   
        //     r->shooter.shooter_first_time = false;
        // }
        // // Hold Right Bumper And firing motor task is off
        // else if (in->mate.bumper_right.held && r->shooter.firing_motor_task == false)  r->shooter.firing_motor_speed = -CFG_SHOOTER_MAX_FIRING_SPEED;
        // // Do Nothing And firing motor task is off
        // else if (r->shooter.firing_motor_task == false && r->shooter.brake == false) 
        // {
        //     r->shooter.firing_motor_speed = 0;
        //     r->shooter.brake = true;
        // }

        // if(in->mate.bumper_left.down)
        // {
        //     {
        //         Task t;
        //         t.type = TASK_SEAT_RING;
        //         t.shooter.delay_timer = 0;
        //         t.shooter.delay_length = 0.05f;
        //         t.shooter.seat_speed_control = -0.8f;
        //         t.shooter.seat_speed_firing = 1;
        //         t.shooter.seat_first = true;
        //         t.shooter.maintain_prev_throttle = true;
        //         pushTask(&r->taskmgr, t);
        //     }
        // }

        // -----------------

        //////// REMOVE AFTER CALIBRATION TEST ////////
        // if(in->mate.big_button.down)
        // {
        //     r->shooter.firing_motor_speed += 0.05f;
        // }

        // //Not During Firing Mode, Right Trigger
        // if (in->mate.trigger_right > 0.01f)
        // {
        //     r->shooter.control_motor_speed = in->mate.trigger_right / 5;
        // }

        // //Not During Firing Mode, Left Trigger
        // else if(in->mate.trigger_left > 0.01f)
        // {
        //     r->shooter.control_motor_speed = -in->mate.trigger_left / 5;
        // }
        // //Not During Firing Mode, Nothing
        // else
        // {
        //     r->shooter.control_motor_speed = 0;
        // } 

        // calibrateShooterFiringMotor(&r->shooter);
    }
    else if(mode == ROBOT_AUTO)
    {

        if(r->auto_first)
        {
            r->auto_first = false;
        }
        
        if ( r->middle_wheels)
        {
            v2 targets[DrivetrainSwerve_Count];
            targets[DrivetrainSwerve_BL] = v2{1, 1};
            targets[DrivetrainSwerve_BR] = v2{-1, 1};
            targets[DrivetrainSwerve_FL] = v2{1, -1};
            targets[DrivetrainSwerve_FR] = v2{-1, -1};
            drivetrainUpdateRawVectors(&r->drivetrain, targets, r->delta_time, true);
        }
    }

    frc::SmartDashboard::PutBoolean("Beam braeakea", r->shooter.beam_break.Get());

    // printf("Just before updates \n");
    updateManager(&r->taskmgr, r);

    // // COMP COMP COMP COMP CoMP UNCOMMENT PLEASE
    updateElevator(&r->elevator, r);
    updateIntake(&r->intake);
    updateShooter(&r->shooter, r);
    updateDrivetrainController(r, &r->drivetrain_controller, &r->drivetrain, r->latest_odometry_frame, r->delta_time);
    
    updatePhoton(&r->photon);
    stepLocaliser(r);

    Pose localiser_pose = r->localiser.pose_estimate;

    frc::Pose2d pose(frc::Translation2d((units::meter_t)localiser_pose.position.x, (units::meter_t) -localiser_pose.position.y), frc::Rotation2d());

    r->field.SetRobotPose(pose);
}

