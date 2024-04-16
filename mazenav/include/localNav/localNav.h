#pragma once

#include <cmath>
#include <thread>

#include "communicator/communicator.h" // For global communication

#include "transformations/tfsys.h" // Everything transform-related

#include "localNav/KinematicDriver.h"
#include "localNav/PathFollower.h"
#include "localNav/MiniController.h"

namespace localNav
{
    void main(communication::Communicator* globComm);
}

