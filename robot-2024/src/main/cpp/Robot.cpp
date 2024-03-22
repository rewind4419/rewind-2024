// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"


#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>



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
    initElevator (&r->elevator);



    r->taskmgr = TaskMgr();
    r->sensor_imu = new AHRS(frc::SPI::Port::kMXP);
    r->integrated_imu_pos = {};
    r->imu_basis = 0;
    r->sensor_imu->ZeroYaw();

    // r->lastCalledState = STATE_NONE;
}
 
void robotModeInit(RobotData *r, RobotMode new_mode)
{

    
    r->taskmgr = TaskMgr{};

    if(new_mode == ROBOT_AUTO)
    {
        resetShooter(&r->shooter);
        resetElevator(&r->elevator);
        if(r->side == 0)
        {
            autoCmd(r, AUTO_BLUE_4_PIECE);
        }
        else if(r->side == 1)
        {
            // autoCmd(r, AUTO_RED_1_PIECE_AUTO);
            autoCmd(r, AUTO_RED_4_PIECE);
        }
    }

    
    if(new_mode == ROBOT_TELEOP)
    {
        resetShooter(&r->shooter);
        resetElevator(&r->elevator);
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
        frc::SmartDashboard::PutNumber("Yeet shooter vel", r->shooter.firing_encoder->GetVelocity());

        

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

        //Allign Straight Code
        // if ( r->input.driver.trigger_left > 0.25)
        // {
        //     r->aligner = ALGN_FORWARD;
        // }
        // if ( r->input.driver.trigger_right > 0.25)
        // {
        //     r->aligner = ALGN_BACKWARD;
        // }

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

        //( r->input.driver.trigger_right - r->input.driver.trigger_left) * CFG_DRIVER_ADJUSTMENT_ROTATION_SENSITIVITY * (curr_speed / CFG_DRIVER_SPEED_NORMAL);

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
                if (fabsf(power_curve + adjustment_rotation) > 0.015)
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



        auto *in = &r->input;

        // in->mate = in->driver;

        if (in->mate.big_button.down)
        {
            r->taskmgr = TaskMgr {};
        }


        // Case that robot is in firing mode
        if(r->shooter.firing_mode && !r->shooter.intake_task && !r->ready_fire_amp)
        {
            // During Firing Mode Left Trigger
            if(in->mate.trigger_left > 0.01f && r->photon.first_aim)
            {
                robotCmd(r, ANGLE_TO_SPEAKER);
            }
            //During Firing Mode Using Right Trigger
            if (in->mate.trigger_right > 0.01f)
            {
                r->intake.intake_speed = in->mate.trigger_right / 3;
                r->shooter.control_motor_speed = in->mate.trigger_right / 3;

            }
            //During Firing Mode, Nothing
            else
            {
                r->intake.intake_speed = 0;
                r->shooter.control_motor_speed = 0;
            }
        }
        // Case that robot is in intake mode
        else if(!r->shooter.intake_task)
        {
            //Not During Firing Mode, Right Trigger
            if (in->mate.trigger_right > 0.01f)
            {
                r->intake.intake_speed = in->mate.trigger_right / 2;
                r->shooter.control_motor_speed = in->mate.trigger_right / 2;
            }

            //Not During Firing Mode, Left Trigger
            else if(in->mate.trigger_left > 0.01f)
            {
                r->intake.intake_speed = -in->mate.trigger_left / 2;
                r->shooter.control_motor_speed = -in->mate.trigger_left / 2;

            }
            //Not During Firing Mode, Nothing
            else
            {
                r->intake.intake_speed = 0;
                r->shooter.control_motor_speed = 0;
            } 
        }
        
        //Return to Rest Position
        if(in->mate.b.down)
        {
            r->ready_fire_amp = false;
            {
                Task t;
                t.type = TASK_ELEVATOR_POSITIONING;
                t.elevator.target_height = 0;
                t.shooter.epsilon = 0.2f;
                pushTask(&r->taskmgr, t);
            }   

            {
                Task t;
                t.type = TASK_SHOOTER_POSITIONING;
                t.shooter.target_angle = 0;
                t.shooter.epsilon = 0.4f;
                pushTask(&r->taskmgr, t);
            }     

        }

        if(in->mate.trigger_right > 0.01 && r->ready_fire_amp)
        {
            r->shooter.amp_mode = true;
        }
        else
        {
            r->shooter.amp_mode = false;
        }

        if(in->mate.x.down)
        {
            robotCmd(r, SHOOTER_DELIVER_SPEAKER);
        }

        if(in->mate.y.down)
        {
            robotCmd(r, SHOOTER_DELIVER_AMP);
        }

        if(in->mate.a.down)
        {
            r->ready_fire_amp = false;
            robotCmd(r, INTAKE_TRANSFER);
        }
      
        // Press Right Trigger And firing motor task is on, 3rd is just to make sure it only queues once
        if (in->mate.trigger_right > 0.01 && r->shooter.firing_motor_task == true && r->shooter.shooter_first_time == true) 
        {
            robotCmd(r, SHOOTER_STOP);
            r->shooter.shooter_first_time = false;
        }
        // Press Right Bumper And firing motor task is on, 3rd is just to make sure it only queues once
        else if(in->mate.bumper_right.down && r->shooter.firing_motor_task == true && r->shooter.shooter_first_time == true) 
        {
            {
                Task t;
                t.type = TASK_SHOOTER_STOP;
                pushTask(&r->taskmgr, t);
            }   
            r->shooter.shooter_first_time = false;
        }
        // Hold Right Bumper And firing motor task is off
        else if (in->mate.bumper_right.held && r->shooter.firing_motor_task == false)  r->shooter.firing_motor_speed = -CFG_SHOOTER_MAX_FIRING_SPEED;
        // Do Nothing And firing motor task is off
        else if (r->shooter.firing_motor_task == false && r->shooter.brake == false) 
        {
            r->shooter.firing_motor_speed = 0;
            r->shooter.brake = true;
        }

        if(in->mate.bumper_left.down)
        {
            {
                Task t;
                t.type = TASK_SEAT_RING;
                t.shooter.delay_timer = 0;
                t.shooter.delay_length = 0.05f;
                t.shooter.seat_speed_control = -0.8f;
                t.shooter.seat_speed_firing = 1;
                t.shooter.seat_first = true;
                t.shooter.maintain_prev_throttle = true;
                pushTask(&r->taskmgr, t);
            }
        }

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

