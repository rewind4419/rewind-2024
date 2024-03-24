#pragma once
#include "fennec/taskmgr.h"
#include "Robot.h"


// Left, Middle, and Right is from the perspective of looking at the speaker from
// the middle of the field

// Median is the pre-pickup point

// CALIB FROM SCHOOL
// constexpr Pose BLUE_NOTE_PREPICKUP_LEFT = {{1.732417, 4.708438}, 4.211621};
// constexpr Pose BLUE_NOTE_PREPICKUP_MIDDLE = {{1.5732417, 5.55657221607}, 4.71238898038};
// constexpr Pose BLUE_NOTE_PREPICKUP_RIGHT = {{1.732417 , 6.220470643215}, -1.07002834641};

// constexpr Pose BLUE_NOTE_PICKUP_LEFT = {{2.351634, 4.316087}, 4.225526};
// constexpr Pose BLUE_NOTE_PICKUP_MIDDLE = {{2.351634, 5.55657221607}, 4.71238898038};
// constexpr Pose BLUE_NOTE_PICKUP_RIGHT = {{2.351634, 6.8705743215}, -1.07002834641};

constexpr Pose BLUE_NOTE_PREPICKUP_LEFT = {{1.804506, 4.746390}, 4.142052};
constexpr Pose BLUE_NOTE_PREPICKUP_MIDDLE = {{1.660223, 5.518912}, 4.673930};
constexpr Pose BLUE_NOTE_PREPICKUP_RIGHT = {{1.912710, 6.383937}, -1.064306};

constexpr Pose BLUE_NOTE_PICKUP_LEFT = {{2.238191, 4.456194}, 4.223248};
constexpr Pose BLUE_NOTE_PICKUP_MIDDLE = {{2.385021, 5.511427}, 3.167182};
constexpr Pose BLUE_NOTE_PICKUP_RIGHT = {{2.384786, 6.666340}, -1.082323};


constexpr Pose RED_NOTE_PREPICKUP_LEFT =  {{14.709731, 6.388793}, 0.988424};
constexpr Pose RED_NOTE_PREPICKUP_MIDDLE = {{14.933145, 5.750772}, 1.626931};
//constexpr Pose RED_NOTE_PREPICKUP_RIGHT = {{14.597740, 4.847893}, 2.318544};
constexpr Pose RED_NOTE_PREPICKUP_RIGHT = {{14.711003, 4.741376}, 2.061367};


constexpr Pose RED_NOTE_PICKUP_LEFT = {{14.224972, 6.823692}, 1.086721};
constexpr Pose RED_NOTE_PICKUP_MIDDLE = {{14.094409, 5.660251}, 1.549997};
constexpr Pose RED_NOTE_PICKUP_RIGHT = {{14.262881, 4.607547}, 2.197760};


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

