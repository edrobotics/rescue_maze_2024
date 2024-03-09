#pragma once

#include <netinet/in.h>
#include <unistd.h>
#include <optional>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>

#include <fusion/Victim.h>

#define VISION_PORT 4242
#define VISION_COMMUNICATION_TIMEOUT_MS 10

class VisionCommunicator
{
    public:
    VisionCommunicator();
    ~VisionCommunicator();


    std::vector<Victim> getVictims();

    private:
    std::optional<std::string> getData();
    std::vector<std::string> split(std::string& split, char deliminator);

    std::optional<char> findChar(std::string& searchString, std::vector<char> chars);
    std::optional<Victim::VictimType> parseVictimType(std::string& str);
    std::optional<int> parseInt(std::string& str);
    std::optional<long> parseLong(std::string& str);

    struct timeval commTimeout{0, VISION_COMMUNICATION_TIMEOUT_MS*1'000};
    int clientSocket;
    int listeningSocket;
};