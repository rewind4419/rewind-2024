#include "localiser.h"
#include "../Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>

void initLocaliser(Localiser_FirstOrderLag* localiser)
{
  std::string side = frc::SmartDashboard::GetString("Auto Init Side", "Blue");
  if(side == "left" || side == "Left" || side == "left " || side == "Left "|| side == "blue", side == "Blue" || side == "blue " || side == "Blue ")
  {
    localiser->pose_estimate.rotation = - M_PI / 2;
  }
  else if(side == "right" || side == "Right" || side == "right " || side == "Right "|| side == "re", side == "Red" || side == "red " || side == "Red ")
  {
    localiser->pose_estimate.rotation = M_PI / 2;
  }

  // Bottom of speaker should be 36.17 inches from april tag
  // Bottom of speaker should be 36.17 inches from april tag
  v2 init_pose = {52.17 * INCH_TO_METER, 218.42 * INCH_TO_METER};
  localiser->pose_estimate.position = init_pose;
}

void stepLocaliser(RobotData* robot)
{

  Localiser_FirstOrderLag* localiser = &robot->localiser;
  OdometryFrame odometry_frame = robot->latest_odometry_frame;
  float imu_rotation = degToRad(robot->sensor_imu->GetYaw());

  if (localiser->first)
  {
    localiser->first = false;
    initLocaliser(localiser);
    localiser->prev_imu = imu_rotation;
  }

  float delta_rot = imu_rotation - localiser->prev_imu;
  localiser->pose_estimate.rotation +=  -1 * delta_rot;
  localiser->prev_imu = imu_rotation;

    frc::SmartDashboard::PutNumber("Localiser X", rotate(odometry_frame.delta_position, -localiser->pose_estimate.rotation).x);
    frc::SmartDashboard::PutNumber("Localiser Y", rotate(odometry_frame.delta_position, -localiser->pose_estimate.rotation).y);
    frc::SmartDashboard::PutNumber("Localiser Rotation", localiser->pose_estimate.rotation);


  float apriltag_first_order_lag_damping = 0.1;
  for (int i = 0; i < robot->photon.global_tags.size(); i++)
  {
    v2 curr_tag_pose = { static_cast<float>(robot->photon.global_tags[i].pose.Y()), static_cast<float>(robot->photon.global_tags[i].pose.X()) };

    frc::SmartDashboard::PutNumber("April Tag Global Pose X", curr_tag_pose.x);
    frc::SmartDashboard::PutNumber("April Tag Global Pose Y", curr_tag_pose.y);

    float curr_tag_rotation = static_cast<float>(robot->photon.global_tags[i].pose.Rotation().Z());

    frc::SmartDashboard::PutNumber("April Tag Global Rotation", curr_tag_rotation);

    // localiser->pose_estimate.position = mix(localiser->pose_estimate.position, 
    //                                         curr_tag_pose, 
    //                                         apriltag_first_order_lag_damping);

    // localiser->pose_estimate.rotation = mix(localiser->pose_estimate.rotation, 
    //                                         , 
    //                                         apriltag_first_order_lag_damping);
    
  }
  // if(robot->photon.n_tags == 0) localiser->pose_estimate.position = localiser->pose_estimate.position + rotate(odometry_frame.delta_position, -localiser->pose_estimate.rotation);
  localiser->pose_estimate.position = localiser->pose_estimate.position + rotate(odometry_frame.delta_position, -localiser->pose_estimate.rotation);

  robot->photon.global_tags.clear();
}