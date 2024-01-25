#pragma once

#include <cmath>

#include "communicator/communicator.h" // For global communication

#include "transformations/tfsys.h" // Everything transform-related
namespace localNav
{
    void main(communication::Communicator* globComm);
}

