#include "photonvis.h"
#include <photon/PhotonUtils.h>
#include "field_layout.h"


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

            photon->tag_rel_robot[working_target.fiducialId - 1] = working_tag_rel_robot;

            frc::Transform3d pose_rel_to_tag = working_tag_rel_robot.Inverse();

            std::optional<frc::Pose3d> tag_pose = photon->aprilTagFieldLayout.GetTagPose(working_target.fiducialId);

            frc::Pose3d robot_pose = tag_pose.value().TransformBy(pose_rel_to_tag);

            // printf("(x, y, z) = (%f, %f, %f)\n", robot_pose.X(), robot_pose.Y(), robot_pose.Z());

            photon->global_pose = robot_pose;
        }
    }
    else photon->n_tags = 0;
}