#pragma once

#include "maths.h"
#include "drivetrain.h"

constexpr v2 ROBOT_LOCAL_FORWARD = { 0, 1 };
constexpr v2 ROBOT_LOCAL_RIGHT   = { 1, 0 };


struct Localiser_ParticleFilter
{
    
};

void stepLocaliser(Localiser_ParticleFilter* localiser, OdometryFrame odometry_frame, Pose imu_pose, Pose april_tag_pose, int april_tags_detected);



struct Localiser_FirstOrderLag
{
  bool first = true;
  Pose pose_estimate;
  float prev_imu;
};

void stepLocaliser(Localiser_FirstOrderLag* localiser, OdometryFrame odometry_frame, float imu_rotation, Pose april_tag_pose, int april_tags_detected, int max_april_tag_count);