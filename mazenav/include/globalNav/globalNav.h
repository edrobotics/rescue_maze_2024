#pragma once

#include <cstdint>
#include "communicator/communicator.h"
#include "globalNav/mazeNavigator.h"

using namespace std;

namespace globalNav
{
    void main(communication::Communicator* communicatorInstance);
}