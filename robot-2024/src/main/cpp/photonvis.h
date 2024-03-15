#include <iostream>
#include <photon/PhotonCamera.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include "fennec/maths.h"


struct PhotonParameters
{
    photon::PhotonCamera april_cam{"front"};

	frc::AprilTagFieldLayout aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

	frc::Transform3d robotToCam = frc::Transform3d( frc::Translation3d(0_m, 0_m, 0_m), frc::Rotation3d(0_rad, 0_rad, 0_rad));

    v3 global_pose;

};

void updatePhoton(PhotonParameters* photon);
