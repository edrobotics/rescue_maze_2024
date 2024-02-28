#pragma once

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <optional>

#define VISION_PORT 4242

class VisionCommunicator
{
    public:
    VisionCommunicator();
    ~VisionCommunicator();

    enum class Victim
    {
        none = -1,
        red = 2,
        h = 2
    };

    std::optional<Victim> getVictim();

    private:
    std::string getData();

    int connectedSocket;
    int listeningSocket;
};