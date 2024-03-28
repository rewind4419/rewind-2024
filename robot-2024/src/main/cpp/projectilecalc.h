#pragma once
#include <vector>

struct RobotData;

struct ProjectileCalculations
{
    std::vector<float> calculated_shooter_angles;
    std::vector<float> calculated_rotational_angles;
    bool tag_7_seen = false;
    bool tag_4_seen = false;};

void updateProjectileCalculations(RobotData* r);
