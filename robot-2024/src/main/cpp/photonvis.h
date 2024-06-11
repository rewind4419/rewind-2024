#pragma once

#include <iostream>
#include <photon/PhotonCamera.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include "fennec/maths.h"
#include <vector>
#include "fennec/config.h"

struct RobotData;

void alignToTag(int tagId, RobotData* robot, bool yaw_align);
void calculateVision(RobotData* robot, int tag_id);

void initVisionCalculations(RobotData* robot);

void initPhoton(RobotData* r);



struct TagPosition
{
    frc::Pose3d pose;
    int tag_id;
};

struct PhotonParameters
{
    photon::PhotonCamera april_cam{"front"};

	frc::AprilTagFieldLayout aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

    frc::Transform3d tag_rel_robot[CFG_APRIL_TAG_COUNT];

    std::vector<TagPosition> global_tags;
    std::vector<TagPosition> global_tags_prev;

    bool first_aim = true;
    bool auto_aim_activated = false;
    bool regression_function = true;
    int n_tags = 0;

    float calculated_pivot_angle = 0;
    float calculated_yaw_angle = 0;

    float pitch_aim_timer = 0;
    float pitch_timer_reset_timer = 0;
};

void updatePhoton(PhotonParameters* photon);

void autoVisionUpdate(RobotData* robot, int tag_id);

