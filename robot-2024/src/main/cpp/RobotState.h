#pragma once
#include "fennec/taskmgr.h"


// Left, Middle, and Right is from the perspective of looking at the speaker from
// the middle of the field

// Median is the pre-pickup point

// CALIB FROM SCHOOL
constexpr Pose BLUE_NOTE_PREPICKUP_LEFT = {{1.43485155105591, 4.716212272644043}, 4.164802074432373};
constexpr Pose BLUE_NOTE_PREPICKUP_MIDDLE = {{1.465522170066833, 5.583795070648193}, 4.676018714904785};
constexpr Pose BLUE_NOTE_PREPICKUP_RIGHT = {{1.43791275024414, 6.643779182434082}, -1.363563537597656};

constexpr Pose BLUE_NOTE_PICKUP_LEFT = {{2.232509851455688, 4.278995513916016}, 4.211971759796143};
constexpr Pose BLUE_NOTE_PICKUP_MIDDLE = {{2.156653983688354, 5.578604698181152}, -1.528661012649536};
constexpr Pose BLUE_NOTE_PICKUP_RIGHT = {{2.076279640197754, 6.877202644348145}, -1.157039880752563};

// constexpr Pose BLUE_NOTE_PREPICKUP_LEFT = {{1.804506, 4.746390}, 4.142052};
// constexpr Pose BLUE_NOTE_PREPICKUP_MIDDLE = {{1.660223, 5.518912}, 4.673930};
// constexpr Pose BLUE_NOTE_PREPICKUP_RIGHT = {{1.912710, 6.383937}, -1.064306};

// constexpr Pose BLUE_NOTE_PICKUP_LEFT = {{2.238191, 4.456194}, 4.223248};
// constexpr Pose BLUE_NOTE_PICKUP_MIDDLE = {{2.385021, 5.511427}, 3.167182};
// constexpr Pose BLUE_NOTE_PICKUP_RIGHT = {{2.384786, 6.666340}, -1.082323};


constexpr Pose RED_NOTE_PREPICKUP_LEFT =  {{14.709731, 6.388793}, 0.988424};
constexpr Pose RED_NOTE_PREPICKUP_MIDDLE = {{14.933145, 5.750772}, 1.626931};
//constexpr Pose RED_NOTE_PREPICKUP_RIGHT = {{14.597740, 4.847893}, 2.318544};
constexpr Pose RED_NOTE_PREPICKUP_RIGHT = {{14.711003, 4.741376}, 2.061367};


constexpr Pose RED_NOTE_PICKUP_LEFT = {{14.224972, 6.823692}, 1.086721};
constexpr Pose RED_NOTE_PICKUP_MIDDLE = {{14.094409, 5.660251}, 1.549997};
constexpr Pose RED_NOTE_PICKUP_RIGHT = {{14.262881, 4.607547}, 2.197760};



enum RobotStateMode
{
    MODE_DEFAULT = 0,

    MODE_AMP,

    MODE_INTAKING,

    MODE_SHOOTING,

    MODE_CLIMBING,
};

struct RobotStateContainer
{
    RobotStateMode robotMode;
};

enum RobotCommand
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
    AUTO_NONE = 0,

    AUTO_4_PIECE,

    AUTO_TEST,

    AUTO_SINGLE_TEST,
};

enum AutoAlliance
{
    A_ALLIANCE_NONE,
    A_ALLIANCE_RED,
    A_ALLIANCE_BLUE
};

struct RobotData;

void robotCmd(RobotData* r, RobotCommand state);

void autoCmd(RobotData* r, AutoState state, AutoAlliance alliance);

