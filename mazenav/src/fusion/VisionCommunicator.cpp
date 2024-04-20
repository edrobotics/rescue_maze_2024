#include "fusion/VisionCommunicator.h"

using namespace std;

void VisionCommunicator::visionServerLooper(communication::Communicator* communicatorInstance)
{
    globComm = communicatorInstance;
    createServerAndBindClient();
    while (true)
    {
        auto bestVictimOptional = getBestVictim();
        if (bestVictimOptional)
        {
            communicatorInstance->logger.logToAll("VisionComm: sending vitim");
            communicatorInstance->victimDataComm.addVictimToStatusQueue(bestVictimOptional.value());
        }
    }
}

void VisionCommunicator::createServerAndBindClient()
{
    struct sockaddr_in address;
    int opt = 1;
    socklen_t addrlen = sizeof(address);
 
    // Creating socket file descriptor
    if ((listeningSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("socket failed");
        return;
    }
 
    // Forcefully attaching socket to the port 4242
    if (setsockopt(listeningSocket, SOL_SOCKET,
                   SO_REUSEADDR | SO_REUSEPORT, &opt,
                   sizeof(opt))) {
        perror("setsockopt");
        return;
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(VISION_PORT);
 
    // Forcefully attaching socket to the port 4242
    if (bind(listeningSocket, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("VisionCommunicator: bind failed");
        return;
    }
    if (listen(listeningSocket, 3) < 0) {
        perror("VisionCommunicator: listen");
        return;
    }
    if ((clientSocket = accept(listeningSocket, (struct sockaddr*)&address, &addrlen)) < 0) {
        perror("VisionCommunicator: accept");
        return;
    }
}

VisionCommunicator::~VisionCommunicator()
{
    // closing the connected socket
    close(clientSocket);
    // closing the listening socket
    close(listeningSocket);
}

std::optional<Victim> VisionCommunicator::getBestVictim() //Remove after linkping
{
    auto victims = getVictims();
    if (victims.empty()) return nullopt;

    return victims.back(); //Latest should be best
}

vector<Victim> VisionCommunicator::getVictims()
{
    vector<Victim> victimDatas;
    string victimData = getData();
    if (victimData.empty()) return victimDatas;

    std::vector<std::string> commands = split(victimData, '!');
    if (victimData[0] != '!') commands.erase(commands.begin()); //Remove first element, as it did not >start< with a '!'

    for (auto i = commands.begin(); i != commands.end(); i++)
    {
        auto constructedVictimOptional = constructVictim(*i);
        if (constructedVictimOptional)
            victimDatas.push_back(constructedVictimOptional.value());
    }
    
    return victimDatas;
}

string VisionCommunicator::getData()
{
    fd_set rfd;
    FD_ZERO(&rfd);
    FD_SET(clientSocket, &rfd);

    ssize_t valread;
    const int bufSize = 1024;
    char buffer[bufSize] = { 0 };

    valread = read(clientSocket, buffer, bufSize - 1); // subtract 1 for the null terminator at the end

    if (valread < 0) {
        perror("VisionCommunicator: read error");
        return "";
    }
    if (valread == 0)
        return "";

    printf("Visioncomm Recived ::%s::\n", buffer);

    return string(buffer);
}


std::optional<Victim> VisionCommunicator::constructVictim(std::string victimData)
{
    if (victimData.size() < 8) return nullopt;
    //Process
    std::vector<std::string> segments = split(victimData, ',');
    struct Victim vicData;

    //Check for !v
    if (!findChar(segments[0], {'v'})) return nullopt;

    //Check for 'p'/'c' - potential/confirmed
    auto detType = findChar(segments[1], {'p', 'c'});
    if (!detType) return nullopt;
    vicData.isConfirmedByVision = (detType.value() == 'c');

    //Check type - 'g'/'y'/'r'/'u'/'s'/'h'
    auto vicType = parseVictimType(segments[2]);
    if (!vicType) return nullopt;
    vicData.victimType = vicType.value();

    //Parsing numbers
    //Check which camera - 0++
    auto cam = parseInt(segments[3]);
    if (!cam) return nullopt;
    vicData.captureCamera = (Victim::RobotCamera)cam.value();
    if (!hasWallInCameraDirection(vicData.captureCamera)) return nullopt;

    //Position X int mm (+/-)
    auto victimX = parseInt(segments[4]);
    if (!victimX) return nullopt;

    //Position Y int mm (+/-)
    auto victimY = parseInt(segments[5]);
    if (!victimY) return nullopt;

    //Unix timestamp
    auto visionTimestamp = parseLong(segments[6]);
    if (!visionTimestamp) return nullopt;
    vicData.captureUnixTimestamp = visionTimestamp.value();
    return vicData;
}

bool VisionCommunicator::hasWallInCameraDirection(Victim::RobotCamera camera)
{
    #warning IMPLEMENT
    std::vector<communication::Walls> wallStates {globComm->poseComm.requestWallStates()};
    //if (onRamp) return false;
    switch (camera)
    {
    case Victim::RobotCamera::FrontCam:
        return true; //return hasFrontWall;
    case Victim::RobotCamera::LeftCam:
        return true; //return hasLeftWall;
    case Victim::RobotCamera::RightCam:
        return true; //return hasRightWall;
    default:
        return false;
    }
}

vector<string> VisionCommunicator::split(string& split, char deliminator)
{
    std::stringstream sStream(split);
    std::string segment;
    std::vector<std::string> segList;

    while(std::getline(sStream, segment, deliminator))
    {
        segList.push_back(segment);
    }
    return segList;
}

optional<char> VisionCommunicator::findChar(string& searchString, vector<char> chars)
{
    for (auto i = searchString.begin(); i != searchString.end(); i++)
    {
        for (auto j = chars.begin(); j != chars.end(); j++)
        {
            if (*i == *j)
            {
                return *j;
            }
        }
    }

    return nullopt;
}

optional<Victim::VictimType> VisionCommunicator::parseVictimType(string& str)
{
    //Check type - 'g'/'y'/'r'/'u'/'s'/'h'
    for (auto i = str.begin(); i != str.end(); i++)
    {
        switch (*i)
        {
        case 'g':
            return Victim::VictimType::Green;
        case 'y':
            return Victim::VictimType::Yellow;
        case 'r':
            return Victim::VictimType::Red;
        case 'u':
            return Victim::VictimType::U;
        case 's':
            return Victim::VictimType::S;
        case 'h':
            return Victim::VictimType::H;
        default:
            break;
        }
    }
    return nullopt;
}

optional<int> VisionCommunicator::parseInt(string& str)
{
    try
    {
        return std::stoi(str);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    return nullopt;
}

optional<long> VisionCommunicator::parseLong(string& str)
{
    try
    {
        return std::stol(str);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return nullopt;
}