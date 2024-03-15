#include <iostream>
#include <photon/PhotonCamera.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include "fennec/maths.h"


struct PhotonParameters
{
    photon::PhotonCamera april_cam{"front"};

	frc::AprilTagFieldLayout aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

    frc::Transform3d tag_rel_robot;

    frc::Transform3d global_pose;
};

void updatePhoton(PhotonParameters* photon);
