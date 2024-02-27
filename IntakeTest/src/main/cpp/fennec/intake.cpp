// #include "intake.h"
// #include "robot.h"
// #include "fennec/maths.h"


// // void intake(Robot* robot, bool out)
// // {
// //     if (out) {
// //         robot->intake.intake_motor.Set(0.4f);
// //     }
// //     else {
// //         robot->intake.intake_motor.Set(-0.4f);
// //     }
// // }

// float intakeGetPose(Robot* robot) {
//     return 0;
// }

// void initIntake(Robot* robot)
// {
    
// 	// frc::SmartDashboard::PutNumber("IntakeOffset", 0.9);
//     float intake_offset = CFG_INTAKE_AXIS_OFFSET; // frc::SmartDashboard::GetNumber("IntakeOffset", CFG_INTAKE_AXIS_OFFSET);

//     float curr_ang = robot->intake.intake_encoder.GetAbsolutePosition() - intake_offset;
//     // if (curr_ang < 0) curr_ang += 1;

//     // curr_ang = fmodf(curr_ang, 1);



//     robot->intake.deliver_angle_offset = 0;
//     robot->intake.sum_angle = curr_ang;
// }

// void updateIntake(Robot* robot)
// {
//     // robot->intake.target_angle = CLAMP(robot->intake.target_angle, -(M_PI/2 - .2), M_PI / 2);
//     {
//         float intake_offset = CFG_INTAKE_AXIS_OFFSET; // frc::SmartDashboard::GetNumber("IntakeOffset", CFG_INTAKE_AXIS_OFFSET);

//         float curr_ang_tmp = robot->intake.intake_encoder.GetAbsolutePosition() - intake_offset;

//         v2 current_facing = rotate({ 0, 1 }, curr_ang_tmp * 2 * M_PI);
        
//         v2 last_facing = rotate({ 0, 1 }, robot->intake.sum_angle * 2 * M_PI);

//         robot->intake.sum_angle += angleBetween(last_facing, current_facing) / (2 * M_PI);

//         float angle_diff = robot->intake.target_angle + robot->intake.deliver_angle_offset - robot->intake.sum_angle;
//         float pid = evalPid(&robot->intake.intake_pid, angle_diff, robot->delta_time);

//         float target_throttle = CLAMP(pid, -CFG_INTAKE_THROTTLE, CFG_INTAKE_THROTTLE);

//         robot->intake.actual_throttle = mix(
//             robot->intake.actual_throttle,
//             target_throttle,
//             0.2f
//         );

//         robot->intake.actual_throttle = CLAMP(robot->intake.actual_throttle, -CFG_INTAKE_THROTTLE, CFG_INTAKE_THROTTLE);
//         robot->intake.axis_motor.Set(robot->intake.actual_throttle);

//         robot->intake.intake_motor.Set(robot->intake.puller_throttle);

//         frc::SmartDashboard::PutNumber("CubeIntake Pos", robot->intake.sum_angle);
//     }
// }
