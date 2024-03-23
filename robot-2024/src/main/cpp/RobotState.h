#pragma once
#include "fennec/taskmgr.h"
#include "Robot.h"


// Left, Middle, and Right is from the perspective of looking at the speaker from
// the middle of the field

// Median is the pre-pickup point

constexpr Pose BLUE_NOTE_PREPICKUP_LEFT = {{1.732417, 4.708438}, 4.211621};
constexpr Pose BLUE_NOTE_PREPICKUP_MIDDLE = {{1.5732417, 5.55657221607}, 4.71238898038};
constexpr Pose BLUE_NOTE_PREPICKUP_RIGHT = {{1.732417 , 6.220470643215}, -1.07002834641};

constexpr Pose BLUE_NOTE_PICKUP_LEFT = {{2.351634, 4.316087}, 4.225526};
constexpr Pose BLUE_NOTE_PICKUP_MIDDLE = {{2.351634, 5.55657221607}, 4.71238898038};
constexpr Pose BLUE_NOTE_PICKUP_RIGHT = {{2.351634, 6.8705743215}, -1.07002834641};


constexpr Pose RED_NOTE_PREPICKUP_LEFT =  {{14.95, 5.64}, M_PI / 2};
constexpr Pose RED_NOTE_PREPICKUP_MIDDLE = {{14.95, 5.64}, M_PI / 2};
constexpr Pose RED_NOTE_PREPICKUP_RIGHT = {{14.95, 5.64}, M_PI / 2};

constexpr Pose RED_NOTE_PICKUP_LEFT = {{14.95, 5.64}, M_PI / 2};
constexpr Pose RED_NOTE_PICKUP_MIDDLE = {{14.95, 5.64}, M_PI / 2};
constexpr Pose RED_NOTE_PICKUP_RIGHT = {{14.95, 5.64}, M_PI / 2};


enum RobotState
{
    STATE_NONE = 0,

    INTAKE_OFF_GROUND,

    INTAKE_TRANSFER,

    INTAKE_OFF_GROUND_WITHOUT_BB,

    SHOOTER_STOP,

    SHOOTER_DELIVER_SPEAKER,

    SHOOTER_DELIVER_AMP,

    ANGLE_TO_SPEAKER,

    CLIMB_POSITIONING, //Comment out if it doesn't work - Nethra

    CLIMBING, //Comment out if it doesn't work - Nethra
};


enum AutoState
{
    AUTO_STATE_NONE = 0,

    AUTO_BLUE_2_PIECE_AUTO,

    AUTO_RED_1_PIECE_AUTO,

    AUTO_TEST,

    AUTO_BLUE_4_PIECE,

    AUTO_RED_4_PIECE,
};


void robotCmd(RobotData* r, RobotState state);

void autoCmd(RobotData* r, AutoState state);

