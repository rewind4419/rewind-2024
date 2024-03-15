#include "photonvis.h"
#include "field_layout.h"
#include <photon/PhotonUtils.h>

void updatePhoton(PhotonParameters* photon)
{
    photon::PhotonPipelineResult result = photon->april_cam.GetLatestResult();

    if(result.HasTargets())
    {
        std::span<const photon::PhotonTrackedTarget> targets = result.GetTargets();
        for(int i = 0; i < targets.size(); i++)
        {
            photon::PhotonTrackedTarget working_target = targets[i];

            photon->tag_rel_robot = working_target.GetBestCameraToTarget();

            frc::Transform3d pose_rel_to_tag = photon->tag_rel_robot.Inverse();

            std::optional<frc::Pose3d> tag_pose = photon->aprilTagFieldLayout.GetTagPose(working_target.fiducialId);

            frc::Pose3d robot_pose = tag_pose.value().TransformBy(pose_rel_to_tag);

            printf("(x, y, z) = (%f, %f, %f)\n", robot_pose.X(), robot_pose.Y(), robot_pose.Z());

            photon->global_pose = robot_pose;
        }
    }
}