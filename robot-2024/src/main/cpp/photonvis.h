#include <iostream>
#include <photon/PhotonCamera.h>
#include "fennec/maths.h"


struct PhotonParameters
{
    photon::PhotonCamera april_cam{"front"};
    v2 global_pose;
};

void updatePhoton(PhotonParameters* photon);
