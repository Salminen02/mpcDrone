/** 
#pragma once
#include "externalController.hpp"


struct simulatorPackage{
    gyro
    accel
    timestamp
}

struct betaflightPackage{
    BLmotor;
    BRmotor;
    FLmotor;
    FRmotor;
    timestamp;
}

class Santerinsimulaattori{
public:
    getSimulatorPackage
    getBetaflightPackage
private:
};
*/