#pragma once

struct Victim
{
    enum class VictimType
    {
        Green = 0,
        Yellow = 1,
        Red = 2,
        U = 0,
        S = 1,
        H = 2
    };
    enum class RobotCamera
    {
        FrontCam = 0,
        RightCam = 1,
        LeftCam = 2,
    };

    VictimType victimType;
    RobotCamera captureCamera;
    bool isConfirmedByVision;
    int xPos;
    int yPos;
    long captureUnixTimestamp;
};