#include "fusion/Imu.h"

Imu::Imu(TeensyCommunicator* communicator)
{
    mtx_general.lock();
    this->communicator = communicator;
    mtx_general.unlock();
}

bool Imu::updateVals()
{
    mtx_general.lock();
    bool retVal {communicator->transData.tsGetIMU(0, vals)};
    if (retVal==true)
    {
        quatVals.real = vals[TransferData::imu_real];
        quatVals.i = vals[TransferData::imu_i];
        quatVals.j = vals[TransferData::imu_j];
        quatVals.k = vals[TransferData::imu_k];
        if (quatVals.real==0 && quatVals.i==0 && quatVals.j==0 && quatVals.k==1)
        {
            // Reset maybe happened
            wasReset = true;
        }
        else
        {
            wasReset = false;
        }
        angles = quaternionToEuler(quatVals);
        // std::cout << radToDeg(angles).z << "\n";
        // mtx_general.unlock();
        // printVals(true);
        // mtx_general.lock();
        // angles = radToDeg(angles)
    }
    mtx_general.unlock();
    // printVals(true);

    return retVal;
}

void Imu::printVals(bool newLine)
{
    mtx_general.lock();
    // std::cout << "real=" << quatVals.real << "  ";
    // std::cout << "i=" << quatVals.i << "  ";
    // std::cout << "j=" << quatVals.j << "  ";
    // std::cout << "k=" << quatVals.k << "  ";
    std::cout << "x= " << std::setfill('0') << std::internal << std::setw(5) << std::fixed << std::setprecision(1) << angles.x*180/M_PI << "  ";
    std::cout << "y= " << std::setfill('0') << std::internal << std::setw(5) << std::fixed << std::setprecision(1) << angles.y*180/M_PI << "  ";
    std::cout << "z= " << std::setfill('0') << std::internal << std::setw(5) << std::fixed << std::setprecision(1) << angles.z*180/M_PI << "  ";

    // std::cout << "x=" << std::fixed << std::setprecision(1) << angles.x << "  ";
    // std::cout << "y=" << std::fixed << std::setprecision(1) << angles.y << "  ";
    // std::cout << "z=" << std::fixed << std::setprecision(1) << angles.z << "  ";
    if (newLine)
    {
    std::cout << "\n";
    }

    mtx_general.unlock();
}

void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]){
    res[0] = atan2( r31, r32 );
    res[1] = asin ( r21 );
    res[2] = atan2( r11, r12 );
}
// You should probably not touch this function ever again, unless you know for certain there is a bug here. This is dark magic... very dark.
Imu::EulerAngle Imu::quaternionToEuler(Quaternion q)
{


    if (true)
    {
        // NEW, WORKING CODE
        // Test with rotation order (robot) XYZ, which should be IMU rotation order YXZ
        EulerAngle localData {};

        double r31 {2*(q.i*q.j + q.real*q.k)};
        double r32 {q.real*q.real - q.i*q.i + q.j*q.j - q.k*q.k};
        localData.x = atan2(r31, r32);

        double r21 {-2*(q.j*q.k - q.real*q.i)};
        localData.y = asin(r21);
        
        double r11 {2*(q.i*q.k + q.real*q.j)};
        double r12 {q.real*q.real - q.i*q.i - q.j*q.j + q.k*q.k};
        localData.z = atan2(r11, r12);

        EulerAngle returnData {};
        returnData.x = -localData.z;
        returnData.y = localData.y;
        returnData.z = localData.x;

        // Verklighet => nuvarande
        // z => y
        // y => -x
        // x => -z

        return returnData;
    }

    if (false)
    {
        // TEST
        // YZX rotation sequence is used for the IMU => XZY for robot (?)

        // Through testing: rotation order seems to be ZYX, but the axis are wrong. X and Z seem flipped
        EulerAngle localData {};

        double r31 {-2*(q.j*q.k - q.real*q.i)};
        double r32 {q.real*q.real - q.i*q.i + q.j*q.j - q.k*q.k};
        localData.x = atan2(r31, r32);

        double r21 {2*(q.i*q.j + q.real*q.k)};
        localData.y = asin(r21);

        double r11 {-2*(q.i*q.k - q.real*q.j)};
        double r12 {q.real*q.real + q.i*q.i - q.j*q.j - q.k*q.k};
        localData.z = atan2(r11, r12);

        EulerAngle returnData {};

        // returnData.x = -localData.y;
        // returnData.y = localData.x;
        // returnData.z = localData.z;

        return localData;

    }

    if (false)
    {
        // ORIGINAL CODE
        
        // YZX rotation sequence is used

        EulerAngle localData {};

        // Code taken/inspired from here: https://web.archive.org/web/20140824191842/http://bediyap.com/programming/convert-quaternion-to-euler-rotations/
        // Note: I switched z and y as they seemed to be swapped. Then it worked perfectly. But why?

        double r31 = -2*(q.j*q.k - q.real*q.i);
        double r32 = q.real*q.real - q.i*q.i + q.j*q.j - q.k*q.k;
        localData.x = atan2(r31, r32);

        double r21 = 2*(q.i*q.j + q.real*q.k);
        localData.z = asin(r21);

        double r11 = -2*(q.i*q.k - q.real*q.j);
        double r12 = q.real*q.real + q.i*q.i - q.j*q.j - q.k*q.k;
        localData.y = atan2(r11, r12);

        EulerAngle returnData {};

        // Change from sensor local coordinate system to global coordinate system conventions. New rotation order is XZY
        returnData.x = -localData.y;
        returnData.y = localData.x;
        returnData.z = localData.z;


        return returnData;

    }


}

Imu::EulerAngle Imu::radToDeg(EulerAngle angle)
{
    EulerAngle retAngle {};
    retAngle.x = angle.x * 180.0/M_PI;
    retAngle.y = angle.y * 180.0/M_PI;
    retAngle.z = angle.z * 180.0/M_PI;
    return retAngle;
}


bool Imu::getWasReset()
{
    std::lock_guard<std::mutex> lock(mtx_general);
    return wasReset;
}