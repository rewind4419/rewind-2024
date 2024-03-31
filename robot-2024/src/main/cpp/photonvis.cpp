#include "photonvis.h"
#include <photon/PhotonUtils.h>
#include "field_layout.h"

#include "Robot.h"

void calculateVision(int tagId, RobotData* robot)
{
    float angularThrottle = 0.0;

    if(robot->photon.n_tags != 0)
    {
        float calculated_throttle = 0;
        float tag_y_dist = static_cast<float>(robot->photon.tag_rel_robot[tagId - 1].Y());
        calculated_throttle = 2 * evalPid(&robot->drivetrain_controller.tag_aligner_pid, tag_y_dist, CFG_DELTA_TIME);
        calculated_throttle = CLAMP(calculated_throttle, -CFG_MAX_TAG_ALIGN_THROTTLE, CFG_MAX_TAG_ALIGN_THROTTLE);
        angularThrottle = calculated_throttle;


    }

    else angularThrottle = 0;

    robot->drivetrain_controller.mode = DRIVECTRL_THROTTLE;
    robot->drivetrain_controller.ctrl.throttle.throttle = robot->global_input_translation;
    robot->drivetrain_controller.ctrl.throttle.angular_throttle = angularThrottle; // Uncomment to enable robot rotational movement


    //Projectile Motion
    v2 vect_to_tag = {static_cast<float>(robot->photon.tag_rel_robot[tagId - 1].Y()), static_cast<float>(robot->photon.tag_rel_robot[tagId - 1].X())};
    float dist_from_tag = length(vect_to_tag);

    frc::SmartDashboard::PutNumber("Dist from tag", dist_from_tag);


    // float init_velocity = 0.00195305 * fabs(robot->shooter.firing_encoder->GetVelocity()) + 1.49364;
    float init_velocity = 12.2177f;
    float shooter_curr_angle = robot->shooter.sum_angle / CFG_SHOOTER_MAX_ANGLE * CFG_SHOOTER_ANGLE_RANGE + CFG_SHOOTER_ANGLE_OFFSET;

    //Add function that takes current angle and calculate offset according to the calculated angle

    float offset;
    shooter_curr_angle += offset;

    frc::SmartDashboard::PutNumber("Current Angle", shooter_curr_angle);
    // float shooter_height = CFG_SHOOTER_RADIUS * sinf( shooter_curr_angle + 0.3542) + CFG_SHOOTER_AXIS_HEIGHT;


    float angle_fudge_factor = 0.5117 * shooter_curr_angle + 0.169995;
    frc::SmartDashboard::PutNumber("Fudge", angle_fudge_factor);

    shooter_curr_angle += angle_fudge_factor;
    float shooter_height = CFG_SHOOTER_RADIUS * sinf( shooter_curr_angle ) + CFG_SHOOTER_AXIS_HEIGHT;
    
    frc::SmartDashboard::PutNumber("Shooter Height", shooter_height * 39.37);

    

    // float shooter_offset = CFG_SHOOTER_DIST_CAM_TO_AXIS - CFG_SHOOTER_RADIUS * cosf(shooter_curr_angle);
    // dist_from_tag += shooter_offset;

    float equation_term_1 = (CFG_GRAVITATIONAL_CONSTANT * std::pow(dist_from_tag, 2)) / std::pow(init_velocity, 2);

    float solved_angle_1 = atan( (dist_from_tag - fabs( sqrtf( std::pow(dist_from_tag, 2) - 2 * equation_term_1 * ( 1/2 * equation_term_1 + CFG_SPEAKER_HEIGHT - shooter_height) ) ) ) / equation_term_1 );
    float solved_angle_2 = atan( (dist_from_tag + fabs( sqrtf( std::pow(dist_from_tag, 2) - 2 * equation_term_1 * ( 1/2 * equation_term_1 + CFG_SPEAKER_HEIGHT - shooter_height) ) ) ) / equation_term_1 );

    float solved_shooter_angle = (solved_angle_1 < solved_angle_2) ? solved_angle_1 : solved_angle_2;

    if (isnanf(solved_shooter_angle) == 0)
    {
        robot->shooter.target_angle = solved_shooter_angle - CFG_SHOOTER_ANGLE_OFFSET; // Uncomment to enable shooter a movement
    }

    frc::SmartDashboard::PutNumber("Aim Calculated Angle", solved_shooter_angle);
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