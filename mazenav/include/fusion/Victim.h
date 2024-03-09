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
        LeftCam = 0,
        FrontCam = 1,
        RightCam = 2
    };

    VictimType whichType;
    RobotCamera whichCamera;
    bool confirmed;
    int x;
    int y;
    long timestamp;
};