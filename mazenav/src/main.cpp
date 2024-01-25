#include <thread>
#include <fusion/fusion.h>
#include <globalNav/globalNav.h>
#include <localNav/localNav.h>

#include "communicator/communicator.h"

// Object used for communication
communication::Communicator comm {};


int main()
{
    
    std::thread fusionT = std::thread(fusion::main, &comm);
    // std::thread globNavT = std::thread(globalNav::main, &comm);
    // std::thread locNavT = std::thread(localNav::main, &comm);

    fusionT.join();
    // globNavT.join();
    // locNavT.join();

    return 0;
}