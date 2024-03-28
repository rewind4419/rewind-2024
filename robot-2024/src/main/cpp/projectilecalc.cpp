#include "projectilecalc.h"
#include "photonvis.h"
#include "Robot.h"
#include "fennec/maths.h"

void updateProjectileCalculations(RobotData* r)
{
    r->proj_calcs.calculated_shooter_angles.clear();

    int blue_tag_id = 7;
    int red_tag_id = 4;
    r->proj_calcs.tag_7_seen = false;
    r->proj_calcs.tag_4_seen = false;
    for(int i = 0; i < r->photon.global_tags.size(); i++)
    {
        if(r->photon.global_tags[i].tag_id == 7) r->proj_calcs.tag_7_seen = true;
        else if(r->photon.global_tags[i].tag_id == 4) r->proj_calcs.tag_4_seen = true;
    }

    v2 vect_to_blue_speaker;
    v2 vect_to_red_speaker;
    float angle_to_blue_speaker;
    float angle_to_red_speaker;

    if(r->proj_calcs.tag_7_seen) 
    {
        vect_to_blue_speaker = {static_cast<float>(r->photon.tag_rel_robot[blue_tag_id - 1].Y()), static_cast<float>(r->photon.tag_rel_robot[blue_tag_id - 1].X())};
        angle_to_blue_speaker = asin( vect_to_blue_speaker.x / vect_to_blue_speaker.y);
    }
    else
    {
        std::optional<frc::Pose3d> loaded_tag_pose = r->photon.aprilTagFieldLayout.GetTagPose(blue_tag_id);
        v2 tag_pose = {static_cast<float>(loaded_tag_pose.value().X()), static_cast<float>(loaded_tag_pose.value().Y())};

        vect_to_blue_speaker = tag_pose - r->localiser.pose_estimate.position;

        float angle_to_tag = atan(vect_to_blue_speaker.y / vect_to_blue_speaker.x);
        angle_to_blue_speaker = leastAngularError(r->localiser.pose_estimate.rotation, angle_to_tag - M_PI / 2); // FIX LATER
    }

    if(r->proj_calcs.tag_4_seen) 
    {
        vect_to_red_speaker = {static_cast<float>(r->photon.tag_rel_robot[red_tag_id - 1].Y()), static_cast<float>(r->photon.tag_rel_robot[red_tag_id - 1].X())};
        angle_to_blue_speaker = asin( vect_to_red_speaker.x / vect_to_red_speaker.y);
    }
    else
    {
        std::optional<frc::Pose3d> loaded_tag_pose = r->photon.aprilTagFieldLayout.GetTagPose(red_tag_id);
        v2 tag_pose = {static_cast<float>(loaded_tag_pose.value().X()), static_cast<float>(loaded_tag_pose.value().Y())};

        vect_to_red_speaker = tag_pose - r->localiser.pose_estimate.position;

        float angle_to_tag = atan(vect_to_red_speaker.y / vect_to_red_speaker.x);
        angle_to_red_speaker = leastAngularError(r->localiser.pose_estimate.rotation, angle_to_tag - M_PI / 2); // FIX LATER
    }

    //Iterate over both blue and red side calculations
    //Blue then red
    for(int i = 0; i < 2; i++)
    {
        float dist_from_tag;
        if(i == 0) dist_from_tag = length(vect_to_blue_speaker);
        else if(i == 1) dist_from_tag = length(vect_to_red_speaker);

        //Projectile Motion
        frc::SmartDashboard::PutNumber("Dist from tag", dist_from_tag);

        // float init_velocity = 0.00195305 * fabs(r->shooter.firing_encoder->GetVelocity()) + 1.49364;
        float init_velocity = 12.2177f;
        float shooter_curr_angle = r->shooter.sum_angle / CFG_SHOOTER_MAX_ANGLE * CFG_SHOOTER_ANGLE_RANGE + CFG_SHOOTER_ANGLE_OFFSET;

        frc::SmartDashboard::PutNumber("Current Angle", shooter_curr_angle);

        float angle_fudge_factor = 0.5117 * shooter_curr_angle + 0.169995;
        shooter_curr_angle += angle_fudge_factor;

        frc::SmartDashboard::PutNumber("Fudge", angle_fudge_factor);

        float shooter_height = CFG_SHOOTER_RADIUS * sinf( shooter_curr_angle ) + CFG_SHOOTER_AXIS_HEIGHT;
        
        frc::SmartDashboard::PutNumber("Shooter Height", shooter_height * 39.37);

        // float shooter_offset = CFG_SHOOTER_DIST_CAM_TO_AXIS - CFG_SHOOTER_RADIUS * cosf(shooter_curr_angle);
        // dist_from_tag += shooter_offset;

        float equation_term_1 = (CFG_GRAVITATIONAL_CONSTANT * std::pow(dist_from_tag, 2)) / std::pow(init_velocity, 2);

        float solved_angle_1 = atan( (dist_from_tag - fabs( sqrtf( std::pow(dist_from_tag, 2) - 2 * equation_term_1 * ( 1/2 * equation_term_1 + CFG_SPEAKER_HEIGHT - shooter_height) ) ) ) / equation_term_1 );
        float solved_angle_2 = atan( (dist_from_tag + fabs( sqrtf( std::pow(dist_from_tag, 2) - 2 * equation_term_1 * ( 1/2 * equation_term_1 + CFG_SPEAKER_HEIGHT - shooter_height) ) ) ) / equation_term_1 );

        float solved_shooter_angle = (solved_angle_1 < solved_angle_2) ? solved_angle_1 : solved_angle_2;


        r->proj_calcs.calculated_shooter_angles.push_back(solved_shooter_angle);
    }

}