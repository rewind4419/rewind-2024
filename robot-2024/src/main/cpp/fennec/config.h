#pragma once
#include "maths.h"

// This is the robot's configuration file
// It will have constants and ID's that we can change all in one place



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
constexpr int CFG_CAN_DRIVETRAIN_DRIVE_MOTOR_BL = 2; // D1
constexpr int CFG_CAN_DRIVETRAIN_DRIVE_MOTOR_FL = 3; // D2
constexpr int CFG_CAN_DRIVETRAIN_DRIVE_MOTOR_FR = 4; // D3
constexpr int CFG_CAN_DRIVETRAIN_DRIVE_MOTOR_BR = 5; // D4

constexpr int CFG_CAN_DRIVETRAIN_STEER_MOTOR_BL = 6; // S1
constexpr int CFG_CAN_DRIVETRAIN_STEER_MOTOR_FL = 7; // S2
constexpr int CFG_CAN_DRIVETRAIN_STEER_MOTOR_FR = 8; // S3
constexpr int CFG_CAN_DRIVETRAIN_STEER_MOTOR_BR = 9; // S4

//            ENCODER NAME                                 Encoder LABEL
constexpr int CFG_CAN_DRIVETRAIN_STEER_ENCODER_BL = 10; // S1
constexpr int CFG_CAN_DRIVETRAIN_STEER_ENCODER_FL = 11; // S2
constexpr int CFG_CAN_DRIVETRAIN_STEER_ENCODER_FR = 12; // S3
constexpr int CFG_CAN_DRIVETRAIN_STEER_ENCODER_BR = 13; // S4


//              DRIVETRAIN MOTOR OFFSET                     OFFSET (in radians)
// constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_FL = 2.495787 + M_PI;
// constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_FR = 2.133767 + M_PI;
// constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_BL = 3.253573 + M_PI;
// constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_BR = 1.512505 + M_PI;


constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_FL = 2.491185 + M_PI;
constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_FR = 2.147573 + M_PI;
constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_BL = 3.261243 + M_PI;
constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_BR = 1.512505 + M_PI;


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



//////////////////////////////// Hank ////////////////////////////////
constexpr int CFG_HANK_MOTOR_SEGMENT_0      = 14;
constexpr int CFG_HANK_MOTOR_SEGMENT_1      = 15;

constexpr int CFG_HANK_MOTOR_SEGMENT_0_2    = 16;
constexpr int CFG_HANK_MOTOR_SEGMENT_1_2    = 17;


constexpr int CFG_HANK_ENCODER_SEGMENT_0    = 0;
constexpr int CFG_HANK_ENCODER_SEGMENT_1    = 1;

// TODO: change later
constexpr float CFG_HANK_SEGMENT_LENGTH_0 = 0.95f; // in metres
constexpr float CFG_HANK_SEGMENT_LENGTH_1 = 0.716f; // in metres
constexpr float CFG_HANK_STAG_CG_DISTANCE = 0.06f;

constexpr float CFG_HANK_ENCODER_SEGMENT_OFFSET_0 = 2.30f; ///2.9f; // 3.56f; // 0.447f; // in radians
constexpr float CFG_HANK_ENCODER_SEGMENT_OFFSET_1 = 3.65f - M_PI; // 4.712201f - M_PI; // 1.619f // in radians

constexpr float MAX_INTEGRAL_1st = 0.75f;
constexpr float MAX_INTEGRAL_2nd = 0.75f;

// Positive rotation is towards the front of the robot




// feed forward estimator:

constexpr float CFG_HANK_MOTOR_STALL_TORQUE = 3.28; // Newton-metres
constexpr float CFG_HANK_SEG0_GEAR_RATIO = 300.0;
constexpr float CFG_HANK_SEG1_GEAR_RATIO = 50.0;

constexpr float CFG_HANK_EXTRUSION_DORD = 0.6382978723;

constexpr float CFG_HANK_SEG0_MASS = CFG_HANK_EXTRUSION_DORD * CFG_HANK_SEGMENT_LENGTH_0;
constexpr float CFG_HANK_SEG1_MASS = CFG_HANK_EXTRUSION_DORD * CFG_HANK_SEGMENT_LENGTH_1;
// constexpr float CFG_HANK_STAG_MASS = 3.674;

constexpr float CFG_HANK_STAG_MASS = 5.5;

//////////////////////////////// Stag ////////////////////////////////
constexpr int CFG_STAG_INTAKE_LEFT = 18;
constexpr int CFG_STAG_INTAKE_RIGHT = 19;
constexpr int CFG_STAG_WRIST = 20;

constexpr int CFG_STAG_WRIST_ENCODER = 2;

constexpr float CFG_STAG_WRIST_LOWBOUND = -2 * M_PI;
constexpr float CFG_STAG_WRIST_HIGHBOUND = 2 * M_PI;
constexpr float CFG_STAG_WRIST_OFFSET = 0.417; // 0.337438; // 0.156f; // encoder offset in 0-1 values

// constexpr float CFG_STAG_WRIST_MAX_THROTTLE = 0.2;
// constexpr float CFG_STAG_INTAKE_SPEED = 1.0;
constexpr float CFG_STAG_INTAKE_SPEED = 0.96;


//////////////////////////////// Shrek ////////////////////////////////

constexpr float CFG_LEMON_LIGHT_FRAME_WIDTH = 0;
constexpr float CFG_LEMON_LIGHT_FRAME_HEIGHT = 0;

//////////////////////////////// Intake ////////////////////////////////

constexpr int CFG_INTAKE_AXIS_MOTOR = 21;
constexpr int CFG_INTAKE_MOTOR = 22;
constexpr int CFG_INTAKE_ENCODER = 3;

constexpr float CFG_INTAKE_AXIS_OFFSET = 0.20f;
constexpr float CFG_INTAKE_THROTTLE = 0.85f;
constexpr float CFG_INTAKE_PULLER_THROTTLE = 0.6f;


constexpr float CFG_INTAKE_RETRACTED_POS = 0.0f;
constexpr float CFG_INTAKE_OUTPUTTED_POS = 0.5;

//////////////////////////////// Localiser ////////////////////////////////
constexpr v2 CFG_APRILTAG_CAMERA_OFFSET = { 0, -13 * INCH_TO_METER };

constexpr int CFG_APRILTAG_HIGHEST_ID = 8;


constexpr v2 CFG_CENTER_OF_GRAVITY = { 0, -0.021 };
constexpr float CFG_CG_ANTIDRIFT_STRENGTH = 1;

constexpr float CFG_DRIVETRAIN_ODO_MULTIPLIER = 1.0f;


///// experimental /////

constexpr bool CFG_ENABLE_ANTISLOP_CHAINS_ON_HANK = false;

constexpr int CFG_LED_STRIP_PIN = 9;
constexpr int CFG_LED_STRIP_LENGTH = 86;
