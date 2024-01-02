#include <fusion/fusion.h>


TeensyCommunicator communicator = TeensyCommunicator(1, 0x69);
TransferData transDat = TransferData{};

void fusion::main()
{
    communicator.initiate();
    
    // communicator.testI2C();

    // transDat.test();

    // transDat.setTof(0, 55);
    // transDat.setTof(5, 69);
    // transDat.setRPM(3, 99);
    // transDat.compose();
    // transDat.decompose();
    // int tof0 = 0;
    // int tof5 = 0;
    // int rpm3 = 0;
    // transDat.getTof(0, tof0);
    // transDat.getTof(5, tof5);
    // transDat.getRPM(3, rpm3);
    // std::cout << "TOF0 has the value: " << tof0 << "\n";
    // std::cout << "TOF5 has the value: " << tof5 << "\n";
    // std::cout << "RPM3 has the value: " << rpm3 << "\n";
    while (true)
    {
        communicator.test();
    }
    
}