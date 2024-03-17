#pragma once

#include <iostream>
#include <photon/PhotonCamera.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include "fennec/maths.h"
#include <vector>
#include "fennec/config.h"



struct PhotonParameters
{
    photon::PhotonCamera april_cam{"front"};

	frc::AprilTagFieldLayout aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

    frc::Transform3d tag_rel_robot[CFG_APRIL_TAG_COUNT];

    frc::Pose3d global_pose;

    bool first_aim = true;
    int n_tags = 0;
};

void updatePhoton(PhotonParameters* photon);
