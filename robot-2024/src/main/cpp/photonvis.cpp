#include "photonvis.h"
#include <photon/PhotonUtils.h>
#include "field_layout.h"

#include "Robot.h"

void initVisionCalculations(RobotData* robot)
{
    robot->photon.regression_function = true;
}

void autoVisionUpdate(RobotData* robot, int tag_id)
{
    if(robot->photon.auto_aim_activated)
    {

        alignToTag(tag_id, robot, false);

        float shooter_curr_angle = robot->shooter.sum_angle / CFG_SHOOTER_MAX_ANGLE * CFG_SHOOTER_ANGLE_RANGE + CFG_SHOOTER_ANGLE_OFFSET;

        if(robot->photon.calculated_pivot_angle - shooter_curr_angle < 0.05 && fabs(robot->photon.calculated_yaw_angle) < 0.05 && fabs(robot->shooter.shooter_encoder->GetVelocity()) < 1000)
        {
            robot->photon.pitch_aim_timer += CFG_DELTA_TIME;
        }

        if(robot->photon.pitch_aim_timer > 0.3) 
        {
            robot->shooter.control_motor_speed = 1;
            robot->photon.pitch_timer_reset_timer += CFG_DELTA_TIME;
        }

        if(robot->photon.pitch_timer_reset_timer > .3)
        {
            robot->photon.pitch_aim_timer = 0;
        }

        if(robot->shooter.beam_break.Get() == true)
        {
            robot->intake.intake_speed = 0.5;
            robot->shooter.control_motor_speed = 1;
        }
    }
}

void alignToTag(int tagId, RobotData* robot, bool yaw_align)
{
    calculateVision(robot, tagId);
    float angularThrottle = 0.0;

    if(yaw_align)
    {
        if(robot->photon.n_tags != 0)
        {

            float calculated_throttle = 2 * evalPid(&robot->drivetrain_controller.tag_aligner_pid, robot->photon.calculated_yaw_angle, CFG_DELTA_TIME);
            calculated_throttle = CLAMP(calculated_throttle, -CFG_MAX_TAG_ALIGN_THROTTLE, CFG_MAX_TAG_ALIGN_THROTTLE);
            angularThrottle = calculated_throttle;
        }
        else angularThrottle = 0;

        robot->drivetrain_controller.mode = DRIVECTRL_THROTTLE;
        robot->drivetrain_controller.ctrl.throttle.throttle = robot->global_input_translation;
        robot->drivetrain_controller.ctrl.throttle.angular_throttle = angularThrottle; // Uncomment to enable robot rotational movement
    }

    if (isnanf(robot->photon.calculated_pivot_angle) == 0)
    {
        robot->shooter.target_angle = robot->photon.calculated_pivot_angle - CFG_SHOOTER_ANGLE_OFFSET; // Uncomment to enable shooter a movement
    }

    frc::SmartDashboard::PutNumber("Aim Calculated Angle", robot->photon.calculated_pivot_angle);
}

void calculateVision(RobotData* robot, int tag_id)
{

    v2 vect_to_tag = {static_cast<float>(robot->photon.tag_rel_robot[tag_id - 1].Y()), static_cast<float>(robot->photon.tag_rel_robot[tag_id - 1].X())};
    float angle_to_speaker = asin( vect_to_tag.x / vect_to_tag.y);

    //Projectile Motion
    float dist_from_tag = length(vect_to_tag);


    // float init_velocity = 0.00195305 * fabs(robot->shooter.firing_encoder->GetVelocity()) + 1.49364;
    float init_velocity = 12.2177f;
    float shooter_curr_angle = robot->shooter.sum_angle / CFG_SHOOTER_MAX_ANGLE * CFG_SHOOTER_ANGLE_RANGE + CFG_SHOOTER_ANGLE_OFFSET;

    //Add function that takes current angle and calculate offset according to the calculated angle

    float offset;
    shooter_curr_angle += offset;

    // frc::SmartDashboard::PutNumber("Current Angle", shooter_curr_angle);
    // float shooter_height = CFG_SHOOTER_RADIUS * sinf( shooter_curr_angle + 0.3542) + CFG_SHOOTER_AXIS_HEIGHT;


    float angle_fudge_factor = 0.5117 * shooter_curr_angle + 0.169995;
    // frc::SmartDashboard::PutNumber("Fudge", angle_fudge_factor);

    shooter_curr_angle += angle_fudge_factor;
    float shooter_height = CFG_SHOOTER_RADIUS * sinf( shooter_curr_angle ) + CFG_SHOOTER_AXIS_HEIGHT;

    dist_from_tag = dist_from_tag * cos(degToRad(25));

    float shooter_offset = CFG_SHOOTER_DIST_CAM_TO_AXIS - CFG_SHOOTER_RADIUS * cosf(shooter_curr_angle);
    dist_from_tag += shooter_offset;

    frc::SmartDashboard::PutNumber("Dist from tag", dist_from_tag * 39.37);

    printf("height = %f\n", shooter_height);

    frc::SmartDashboard::PutNumber("Current Height", shooter_height * 39.37);

    float equation_term_1 = (CFG_GRAVITATIONAL_CONSTANT * std::pow(dist_from_tag, 2)) / std::pow(init_velocity, 2);

    float solved_angle_1 = atan( (dist_from_tag - fabs( sqrtf( std::pow(dist_from_tag, 2) - 2 * equation_term_1 * ( 1/2 * equation_term_1 + CFG_SPEAKER_HEIGHT - shooter_height) ) ) ) / equation_term_1 );
    float solved_angle_2 = atan( (dist_from_tag + fabs( sqrtf( std::pow(dist_from_tag, 2) - 2 * equation_term_1 * ( 1/2 * equation_term_1 + CFG_SPEAKER_HEIGHT - shooter_height) ) ) ) / equation_term_1 );

    float solved_shooter_angle = (solved_angle_1 < solved_angle_2) ? solved_angle_1 : solved_angle_2;

    float solved_angle_after_regression_function = 0.878567 * solved_shooter_angle + 0.158185;


    // if(robot->photon.regression_function)
    // {
    //     robot->photon.calculated_pivot_angle = solved_angle_after_regression_function;
    // }
    // else robot->photon.calculated_pivot_angle = solved_shooter_angle;
    robot->photon.calculated_pivot_angle = solved_shooter_angle;

    robot->photon.calculated_yaw_angle = angle_to_speaker;

    frc::SmartDashboard::PutNumber("YAW CHECK", angle_to_speaker);

    frc::SmartDashboard::PutNumber("Calculated Angle", solved_angle_after_regression_function);


}

void updatePhoton(PhotonParameters* photon)
{
    photon::PhotonPipelineResult result = photon->april_cam.GetLatestResult();

    if(result.HasTargets())
    {
        std::span<const photon::PhotonTrackedTarget> targets = result.GetTargets();
        photon->n_tags = targets.size();

        for(int i = 0; i < targets.size(); i++)
        {
            photon::PhotonTrackedTarget working_target = targets[i];

            frc::Transform3d working_tag_rel_robot = working_target.GetBestCameraToTarget();
            // printf("(x, y, z) = (%f, %f, %f)\n", working_tag_rel_robot.X(), working_tag_rel_robot.Y(), working_tag_rel_robot.Z());

            photon->tag_rel_robot[working_target.fiducialId - 1] = working_tag_rel_robot;

            frc::Transform3d pose_rel_to_tag = working_tag_rel_robot.Inverse();

            std::optional<frc::Pose3d> tag_pose = photon->aprilTagFieldLayout.GetTagPose(working_target.fiducialId);

            frc::Pose3d robot_pose = tag_pose.value().TransformBy(pose_rel_to_tag);

            // printf("(x, y, z) = (%f, %f, %f)\n", robot_pose.X(), robot_pose.Y(), robot_pose.Z());

            photon->global_tags.push_back( TagPosition {robot_pose, working_target.fiducialId});
        }
    }
    else photon->n_tags = 0;
}