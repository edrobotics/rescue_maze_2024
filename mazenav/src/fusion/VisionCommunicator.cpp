#include "fusion/VisionCommunicator.h"

using namespace std;

VisionCommunicator::VisionCommunicator()
{
    struct sockaddr_in address;
    int opt = 1;
    socklen_t addrlen = sizeof(address);
    const char* hello = "Hello from server";
 
    // Creating socket file descriptor
    if ((listeningSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
 
    // Forcefully attaching socket to the port 4242
    if (setsockopt(listeningSocket, SOL_SOCKET,
                   SO_REUSEADDR | SO_REUSEPORT, &opt,
                   sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(VISION_PORT);
 
    // Forcefully attaching socket to the port 4242
    if (bind(listeningSocket, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("VisionCommunicator: bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(listeningSocket, 3) < 0) 
    {
        perror("VisionCommunicator: listen");
        exit(EXIT_FAILURE);
    }
    if ((connectedSocket = accept(listeningSocket, (struct sockaddr*)&address, &addrlen)) < 0) 
    {
        perror("VisionCommunicator: accept");
        exit(EXIT_FAILURE);
    }
}

VisionCommunicator::~VisionCommunicator()
{
    // closing the connected socket
    close(connectedSocket);
    // closing the listening socket
    close(listeningSocket);
}

optional<VisionCommunicator::Victim> VisionCommunicator::getVictim()
{
    optional<Victim> result;

    perror("not done");
    return result;
}

string VisionCommunicator::getData()
{
    ssize_t valread;
    const int bufSize = 1024;
    char buffer[bufSize] = { 0 };

    valread = read(connectedSocket, buffer, bufSize - 1); // subtract 1 for the null terminator at the end

    if (valread == -1)
    {
        perror("VisionCommunicator: read error");
        exit (EXIT_FAILURE);
    }

    printf("Recived ::%s::\n", buffer);

    return (string)("%s", buffer);
}