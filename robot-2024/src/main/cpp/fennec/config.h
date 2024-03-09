#pragma once
#include "maths.h"

// This is the robot's configuration file
// It will have constants and ID's that we can change all in one place

constexpr float CFG_DELTA_TIME = 0.02f;

//////////////////////////////// Drivetrain ////////////////////////////////


//////////////////
//3    FRONT   1//
//              //
//             R//
//L            I//
//E            G//
//F            H//
//T            T//
//              //
//4    BACK    2//
//////////////////`


//            MOTOR NAME                                MOTOR LABEL
constexpr int CFG_CAN_DRIVETRAIN_DRIVE_MOTOR_BL = 2; // D4
constexpr int CFG_CAN_DRIVETRAIN_DRIVE_MOTOR_FL = 3; // D3 CHANGE TO ID 3
constexpr int CFG_CAN_DRIVETRAIN_DRIVE_MOTOR_FR = 8; // D1
constexpr int CFG_CAN_DRIVETRAIN_DRIVE_MOTOR_BR = 5; // D2

constexpr int CFG_CAN_DRIVETRAIN_STEER_MOTOR_BL = 6; // S4
constexpr int CFG_CAN_DRIVETRAIN_STEER_MOTOR_FL = 7; // S3
constexpr int CFG_CAN_DRIVETRAIN_STEER_MOTOR_FR = 4; // S1
constexpr int CFG_CAN_DRIVETRAIN_STEER_MOTOR_BR = 9; // S2

//            ENCODER NAME                                 Encoder LABEL
constexpr int CFG_CAN_DRIVETRAIN_STEER_ENCODER_BL = 13; // S4
constexpr int CFG_CAN_DRIVETRAIN_STEER_ENCODER_FL = 12; // S3
constexpr int CFG_CAN_DRIVETRAIN_STEER_ENCODER_FR = 10; // S1
constexpr int CFG_CAN_DRIVETRAIN_STEER_ENCODER_BR = 11; // S2


//              DRIVETRAIN MOTOR OFFSET                     OFFSET (in radians)
// constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_FL = 2.495787 + M_PI;
// constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_FR = 2.133767 + M_PI;
// constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_BL = 3.253573 + M_PI;
// constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_BR = 1.512505 + M_PI;


constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_FL = 3.701496 + M_PI;
constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_FR = 3.232098 + M_PI;
constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_BL = 6.203418 + M_PI;
constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_BR = 2.528000 + M_PI;


// fl -- 2.495787
// fr -- 2.133767
// bl -- 3.253573
// br -- 1.512505




constexpr float CFG_DRIVETRAIN_DISTANCE_BETWEEN_WHEELS_HORIZONTAL = 26 * INCH_TO_METER;
constexpr float CFG_DRIVETRAIN_DISTANCE_BETWEEN_WHEELS_VERTICAL   = 26 * INCH_TO_METER;

// NOTE: drivetrain gear ratios are measured in Motor Rotations : Final Wheel Rotations

constexpr float CFG_DRIVETRAIN_DRIVE_RATIO      = 6.75; // gear ratio of the drive wheel
constexpr float CFG_DRIVETRAIN_STEER_RATIO      = 12.8; // gear ratio of the steer motor
constexpr float CFG_DRIVETRAIN_WHEEL_RADIUS     = 4; // measured in inches


constexpr float CFG_DRIVETRAIN_TARGET_VECTOR_EPSILON = 0.05; // if the move vector is below a certain amount, dont bother moving the wheel to avoid super small numbers causing weird rotations


constexpr float CFG_DRIVETRAIN_ANTIDRIFT = 0.0; // @Try, lets see if 0.1 works?

//////////////////////////////// Driver ////////////////////////////////
// constexpr float CFG_DRIVER_SPEED_NORMAL  = 6 * 0.7;
// constexpr float CFG_DRIVER_SPEED_SURGERY = 6 * 0.59;
// constexpr float CFG_DRIVER_SPEED_SPRINT  = 6 * 1;

// constexpr float CFG_DRIVER_SPEED_SURGERY_ROT = 10;
// constexpr float CFG_DRIVER_SPEED_NORMAL_ROT = 20;
// constexpr float CFG_DRIVER_SPEED_SPRINT_ROT = 20;


// old
constexpr float CFG_DRIVER_SPEED_NORMAL  = 0.7;
constexpr float CFG_DRIVER_SPEED_NORMAL_ROT = 0.7;

constexpr float CFG_DRIVER_SPEED_SURGERY = 0.59;
constexpr float CFG_DRIVER_SPEED_SURGERY_ROT = 0.4;

constexpr float CFG_DRIVER_SPEED_SPRINT  = 1;
constexpr float CFG_DRIVER_SPEED_SPRINT_ROT = 0.7;




constexpr float CFG_DRIVER_ABSOLUTE_ROTATION_REACTIVENESS = 0.8; // 0.5 ....  higher reactivity = closer to 0, straight reactiveness curve = 1

constexpr float CFG_DRIVER_ADJUSTMENT_ROTATION_SENSITIVITY = 0.5;


//////////////////////////////// Intake ////////////////////////////////

constexpr int CFG_INTAKE_MOTOR = 10;

constexpr int CFG_INTAKE_BB_DIO = 0;

constexpr float CFG_INTAKE_MAX_SPEED = 1;



//////////////////////////////// Shooter ////////////////////////////////

constexpr int CFG_SHOOTER_CONTROL_MOTOR = 30;
constexpr int CFG_SHOOTER_FIRING_MOTOR = 31;

constexpr int CFG_SHOOTER_AXIS_LEFT = 21;
constexpr int CFG_SHOOTER_AXIS_RIGHT = 22;
constexpr int CFG_SHOOTER_AXIS_MOTOR_COUNT = 2;

constexpr int CFG_SHOOTER_ENCODER = 23;
constexpr int CFG_SHOOTER_BB_DIO = 1;

constexpr float CFG_SHOOTER_AXIS_OFFSET = 0;
constexpr float CFG_SHOOTER_AXIS_THROTTLE = 0.85f;
constexpr float CFG_SHOOTER_CONTROL_MAX_SPEED = 1;
constexpr float CFG_SHOOTER_MAX_ANGLE = 0; // CHANGE!!!!!!!!!
constexpr float CFG_SHOOTER_ANGLE_RANGE = M_PI/4;


//////////////////////////////// Localiser ////////////////////////////////
constexpr v2 CFG_APRILTAG_CAMERA_OFFSET = { 0, -13 * INCH_TO_METER };

constexpr int CFG_APRILTAG_HIGHEST_ID = 8;


constexpr v2 CFG_CENTER_OF_GRAVITY = { 0, -0.021 };
constexpr float CFG_CG_ANTIDRIFT_STRENGTH = 1;

constexpr float CFG_DRIVETRAIN_ODO_MULTIPLIER = 1.0f;


///// experimental /////

constexpr int CFG_LED_STRIP_PIN = 9;
constexpr int CFG_LED_STRIP_LENGTH = 86;
