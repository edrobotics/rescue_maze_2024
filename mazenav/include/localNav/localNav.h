#pragma once

#include <cmath>

#include "communicator/communicator.h" // For global communication

#include "transformations/tfsys.h" // Everything transform-related

#include "localNav/KinematicDriver.h"

namespace localNav
{
    void main(communication::Communicator* globComm);
}

