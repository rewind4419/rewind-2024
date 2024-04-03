#pragma once

#include <iostream>
#include <photon/PhotonCamera.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include "fennec/maths.h"
#include <vector>
#include "fennec/config.h"

struct RobotData;

void alignToTag(int tagId, RobotData* robot);
void calculateVision(RobotData* robot, int tag_id);

struct TagPosition
{
    frc::Pose3d pose;
    int tag_id;
};

struct PhotonParameters
{
    photon::PhotonCamera april_cam{"back"};

	frc::AprilTagFieldLayout aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

    frc::Transform3d tag_rel_robot[CFG_APRIL_TAG_COUNT];

    std::vector<TagPosition> global_tags;

    bool first_aim = true;
    int n_tags = 0;

    float calculated_pivot_angle = 0;
    float calculated_yaw_angle = 0;
};

void updatePhoton(PhotonParameters* photon);
