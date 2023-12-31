#include <fusion/fusion.h>


TeensyCommunicator communicator = TeensyCommunicator(1, 0x69);
TransferData transDat = TransferData{};

void fusion::main()
{
    // communicator.initiate();
    
    // communicator.testI2C();
    
    transDat.setTof(0, 69);
    transDat.printInt();
    std::cout << "First TOF Read: " << transDat.getTof(0) << "\n";
    transDat.compose();
    transDat.decompose();
    std::cout << "Second TOF Read: " << transDat.getTof(0) << "\n";
}