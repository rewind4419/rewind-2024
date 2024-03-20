#include "localiser.h"

// Particle Filter


// First order lag

#include <frc/smartdashboard/SmartDashboard.h>


void stepLocaliser(Localiser_FirstOrderLag* localiser, OdometryFrame odometry_frame, float imu_rotation, Pose april_tag_pose, int april_tags_detected, int max_april_tag_count)
{
  // drivetrain odometry
  localiser->pose_estimate.position = localiser->pose_estimate.position + rotate(odometry_frame.delta_position, -localiser->pose_estimate.rotation);
  // localiser->pose_estimate.rotation = localiser->pose_estimate.rotation + odometry_frame.delta_rotation;

    frc::SmartDashboard::PutNumber("YeetX", rotate(odometry_frame.delta_position, localiser->pose_estimate.rotation).x);
    frc::SmartDashboard::PutNumber("YeetY", rotate(odometry_frame.delta_position, localiser->pose_estimate.rotation).y);
        frc::SmartDashboard::PutNumber("Delta Position x", odometry_frame.delta_position.x);
    frc::SmartDashboard::PutNumber("Delta Position y", odometry_frame.delta_position.y);
    frc::SmartDashboard::PutNumber("Rotation", localiser->pose_estimate.rotation);


  if (localiser->first)
  {
    localiser->first = false;
    localiser->prev_imu = imu_rotation;
  }

  float delta_rot = imu_rotation - localiser->prev_imu;
  localiser->pose_estimate.rotation +=  -1 * delta_rot;
  localiser->prev_imu = imu_rotation;

  // if(delta_rot < 3.0)
  // {
  //   localiser->
  // }


  // if()

  // localiser->pose_estimate.rotation = fmod(localiser->pose_estimate.rotation, M_PI);
  // if (localiser->pose_estimate.rotation < 0) localiser->pose_estimate.rotation += 2 * M_PI;


  // april_tag_pose.rotation = fmod(april_tag_pose.rotation, 2 * M_PI);
  // if (april_tag_pose.rotation < 0) april_tag_pose.rotation += 2 * M_PI;

  // april tag fusion
  float apriltag_first_order_lag_damping = 0;
  if (april_tags_detected > 0)
  {
    apriltag_first_order_lag_damping = 0.6; // + 0.1 * april_tags_detected / (float)max_april_tag_count;
  }

    // printf("cheese %f\n", apriltag_first_order_lag_damping);

  localiser->pose_estimate.position = mix(localiser->pose_estimate.position, 
                                           april_tag_pose.position, 
                                           apriltag_first_order_lag_damping);

  localiser->pose_estimate.rotation = mix(localiser->pose_estimate.rotation, 
                                           april_tag_pose.rotation, 
                                           apriltag_first_order_lag_damping);




  // localiser->pose_estimate.rotation = mix(localiser->pose_estimate.rotation, 
  //                                          imu_rotation, 
  //                                          );
}