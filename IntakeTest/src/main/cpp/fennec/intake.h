// #pragma once 

// #include "rev/CANSparkMax.h"
// #include <frc/DutyCycleEncoder.h>

// #include "fennec/pid.h"
// #include "fennec/maths.h"
// #include "config.h"

// struct Robot; 
// struct Intake 
// {
//     rev::CANSparkMax axis_motor     { CFG_INTAKE_AXIS_MOTOR, rev::CANSparkMax::MotorType::kBrushless };
//     rev::CANSparkMax intake_motor     { CFG_INTAKE_MOTOR, rev::CANSparkMax::MotorType::kBrushless };

//     frc::DutyCycleEncoder intake_encoder { CFG_INTAKE_ENCODER };

//     PID intake_pid = { .kP = 1.25f, .kI = 0, .kD = 0,  .errorAccum = 0, .lastError = 0 };

//     float target_angle = 0;
//     float deliver_angle_offset = 0;

//     float sum_angle = 0;


//     float actual_throttle = 0.0f;

//     float puller_throttle = 0.0f;
// };

// void initIntake(Robot* robot);
// void intake(Robot* robot, bool out);
// void updateIntake(Robot* robot);
// float intakeGetPose(Robot* robot);

