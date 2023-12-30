#include <fusion/fusion.h>

TeensyCommunicator communicator = TeensyCommunicator(1, 0x69);

void fusion::main()
{
    communicator.initiate();
    
    communicator.testI2C();
}