#include <fusion/fusion.h>


TeensyCommunicator communicator {1, 0x69};

Imu imu0 {&communicator};

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
    // transDat.getTof(0, tof0);https://www.youtube.com/watch?v=9keeDyFxzY4
    // transDat.getTof(5, tof5);
    // transDat.getRPM(3, rpm3);
    // std::cout << "TOF0 has the value: " << tof0 << "\n";
    // std::cout << "TOF5 has the value: " << tof5 << "\n";
    // std::cout << "RPM3 has the value: " << rpm3 << "\n";
    while (true)
    {
        // Note: Not really correct, but I do not want to deal with multithreading right now.
        communicator.runLoop();
        imu0.updateVals();
        imu0.printVals(true);
    }
    
}