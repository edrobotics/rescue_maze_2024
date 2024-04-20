#pragma once

#include <netinet/in.h>
#include <unistd.h>
#include <optional>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>

#include "fusion/Victim.h"
#include "communicator/communicator.h"

#define VISION_PORT 4242
#define VISION_COMMUNICATION_TIMEOUT_MS 10

class VisionCommunicator
{
    public:
    ~VisionCommunicator();

    void visionServerLooper(communication::Communicator* communicatorInstance);

    private:
    communication::Communicator* globComm {nullptr};
    std::optional<Victim> getBestVictim();

    void createServerAndBindClient();

    std::vector<Victim> getVictims();

    std::string getData();
    std::optional<Victim> constructVictim(std::string victimData);
    bool hasWallInCameraDirection(Victim::RobotCamera camera);

    std::vector<std::string> split(std::string& split, char deliminator);

    std::optional<char> findChar(std::string& searchString, std::vector<char> chars);
    std::optional<Victim::VictimType> parseVictimType(std::string& str);
    std::optional<int> parseInt(std::string& str);
    std::optional<long> parseLong(std::string& str);

    bool couldConnect = false;
    struct timeval commTimeout{0, VISION_COMMUNICATION_TIMEOUT_MS*1'000};
    int clientSocket;
    int listeningSocket;
};