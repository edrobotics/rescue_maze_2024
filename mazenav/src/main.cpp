#include <iostream>
#include <thread>
#include <fusion/fusion.h>
#include <globalNav/globalNav.h>
#include <localNav/localNav.h>

#include "communicator/communicator.h"
#include "fusion/PiAbstractor.h"

// Object used for communication
communication::Communicator comm {};

PiAbstractor piAbs {};


int main()
{
    piAbs.init();
    
    std::thread fusionT = std::thread(fusion::main, &comm, &piAbs);
    // std::thread globNavT = std::thread(globalNav::main, &comm);
    // std::thread locNavT = std::thread(localNav::main, &comm, &piAbs);

    fusionT.join();
    // globNavT.join();
    // locNavT.join();

    return 0;
}