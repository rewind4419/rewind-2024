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
            frc::Transform3d pose_rel_to_robot = working_target.GetBestCameraToTarget();
            v2 pose_rel_to_tag = -v2{ static_cast<float>(pose_rel_to_robot.Y()) /* Horizontal axis */, static_cast<float>(pose_rel_to_robot.X()) /* Depth Axis */ };

            AprilTagAnchor tag_anchor = APRIL_TAG_ANCHORS[working_target.fiducialId - 1];

            float angle_of_tag = angleBetween( {1, 0}, tag_anchor.normal);

            float angle_tag_to_robot = degToRad(working_target.GetYaw());

            float total_tag_angle = angle_of_tag + angle_tag_to_robot; // Test, TO BE CHANGED

            v2 actual_pose_rel_tag = rotate(pose_rel_to_tag, total_tag_angle);

            photon->global_pose = tag_anchor.location + actual_pose_rel_tag;
            
        }
    }
}