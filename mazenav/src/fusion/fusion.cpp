#include <fusion/fusion.h>


TeensyCommunicator tComm {1, 0x69};

MotorControllers motors {&tComm};
Sensors sensors {&tComm};

void fusion::main(communication::Communicator* globComm)
{
    tComm.initiate();

    // Create hardware communication thread. Will run indefinetely
    std::thread hardwareCommunicator(&TeensyCommunicator::runLoopLooper, &tComm);
    
    while (true)
    {
        motors.setSpeeds = MotorControllers::MotorSpeeds(50, 50, 50, 50);
        motors.updateVals();

        std::chrono::time_point timeFlag = std::chrono::steady_clock::now();
        while(std::chrono::steady_clock::now()-timeFlag < std::chrono::milliseconds(1000))
        {
            motors.updateVals();
            sensors.update();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            motors.printSpeeds();
        }


        motors.setSpeeds = MotorControllers::MotorSpeeds(0, 0, 0, 0);
        motors.updateVals();

        timeFlag = std::chrono::steady_clock::now();
        while(std::chrono::steady_clock::now()-timeFlag < std::chrono::milliseconds(1000))
        {
            motors.updateVals();
            sensors.update();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            motors.printSpeeds();
        }
        // Update the sensors

        // sensors.print();

        // Update the pose (To be done)
    }
    
}