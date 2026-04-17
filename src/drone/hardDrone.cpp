#include "hardDrone.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility>
#include <iostream>
#include <random>
#include <cmath>

using Eigen::Vector2f;

HardDrone::HardDrone()
    : BLPropeller(Vector2f{-squarePropellerDrone, -squarePropellerDrone}),
      BRPropeller(Vector2f{squarePropellerDrone, -squarePropellerDrone}),
      FLPropeller(Vector2f{-squarePropellerDrone, squarePropellerDrone}),
      FRPropeller(Vector2f{squarePropellerDrone, squarePropellerDrone})
    // FL(-X,+Y)   FR(+X,+Y)
    //      \     /
    //       \   /
    //        \ /
    //       /   \
    // BL(-X,-Y)  BR(+X,-Y)
    //
{
    setOrientation(0, 0, 0);
    
    FRPropeller.RPS = 0;
    FLPropeller.RPS = 0;
    BRPropeller.RPS = 0;
    BLPropeller.RPS = 0;
}

void HardDrone::setOrientation(float roll, float pitch, float yaw)
{
    Eigen::AngleAxisf rollAA(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAA(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAA(yaw, Eigen::Vector3f::UnitZ());
    orientationQ = yawAA * pitchAA * rollAA;
}

float HardDrone::propellerForce(const Propeller& propeller)
{
    float RPS = propeller.RPS;
    float D4 = propeller.D4;
    float Ct = propeller.Ct;
    
    return Ct * rho * RPS * RPS * D4; 
}

float HardDrone::propellerForces()
{
    float p1 = propellerForce(BLPropeller);
    float p2 = propellerForce(BRPropeller);
    float p3 = propellerForce(FLPropeller);
    float p4 = propellerForce(FRPropeller);

    return p1 + p2 + p3 + p4;
}

Vector3f HardDrone::propellerMomentLocal(const Propeller& propeller)
{
    float forceS = propellerForce(propeller);
    Vector3f forceVec = forceS * Vector3f{0,0,1};
    Vector3f r = Vector3f(propeller.coord.x(), propeller.coord.y(),0);
    return r.cross(forceVec);
}

Vector3f HardDrone::propellerMomentsLocal()
{
    Vector3f moment1 = propellerMomentLocal(BLPropeller);
    Vector3f moment2 = propellerMomentLocal(BRPropeller);
    Vector3f moment3 = propellerMomentLocal(FLPropeller);
    Vector3f moment4 = propellerMomentLocal(FRPropeller);

    return moment1 + moment2 + moment3 + moment4;
}

float generateWhiteNoise(float amplitude = 0.1f)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::normal_distribution<float> dist(0.0f, amplitude);
    return dist(gen);
}

void updatePropellerRPS(Propeller& propeller){
    propeller.RPS = 165 + generateWhiteNoise();
}

void HardDrone::move(float dt)
{
    float _propellerForces = propellerForces();

    Vector3f localUp{0.0f, 0.0f, 1.0f};
    Vector3f propellerForceLocal = _propellerForces * localUp;

    // rotate local force to world using quaternion
    Vector3f propellerForceWorld = orientationQ * propellerForceLocal;

    Vector3f gravityForce{0.0f, 0.0f, -9.81f * m};
    Vector3f totalForce = propellerForceWorld + gravityForce;

    acc = totalForce / m;
    vel += acc * dt;
    position += vel * dt;

    //////// rotation /////////
    
    Vector3f gyroscopicTorque = angularVel.cross(I.cast<float>().cwiseProduct(angularVel));  // almost insignificant
    Vector3f yxMomentLocal = propellerMomentsLocal();
    Vector3f _zMomentLocal = totalPropellerMomentZ();
    
    Vector3f totalMomentLocal = yxMomentLocal + _zMomentLocal - gyroscopicTorque;

    Vector3f totalMomentWorld = orientationQ * totalMomentLocal;
    
    // angularAcc = Vector3f(moment.x/inertia.x, moment.y/inertia.y ... )
    angularAcc = Vector3f(totalMomentWorld.x()/I.cast<float>().x(),totalMomentWorld.cast<float>().y()/I.y(), totalMomentWorld.cast<float>().z()/I.z());
    angularVel += angularAcc * dt;

    Eigen::Quaternionf omega_quat(0, angularVel.x(), angularVel.y(), angularVel.z());

    Eigen::Quaternionf dq = omega_quat * orientationQ;
    dq.coeffs() *= 0.5f * dt;

    orientationQ.coeffs() += dq.coeffs();
    orientationQ.normalize();

}

DroneBodyFrame HardDrone::getBodyFrame()
{
    // Paikalliset akselit (drone body frame)
    Eigen::Vector3f x_local(1, 0, 0); // eteen
    Eigen::Vector3f y_local(0, 1, 0); // oikealle
    Eigen::Vector3f z_local(0, 0, 1); // ylös

    // Muunnetaan maailmakoordinaatteihin kvaternionilla
    DroneBodyFrame frame;
    frame.X_forward = orientationQ * x_local;
    frame.Y_right   = orientationQ * y_local;
    frame.Z_up      = orientationQ * z_local;
    return frame;
}

Eigen::Quaternionf HardDrone::getOrienatationQcontroller()
{
    // Lisää kohina kvaternioniin
    float noiseAmplitude = 0.01f;
    float noisex = generateWhiteNoise(noiseAmplitude);
    float noisey = generateWhiteNoise(noiseAmplitude);
    float noisez = generateWhiteNoise(noiseAmplitude);
    float noisew = generateWhiteNoise(noiseAmplitude);
    
    Eigen::Quaternionf noisyQ = orientationQ;
    noisyQ.x() += noisex;
    noisyQ.y() += noisey;
    noisyQ.z() += noisez;
    noisyQ.w() += noisew;
    noisyQ.normalize();
    
    return noisyQ;
}

std::tuple<float, float, float, float> HardDrone::getPropellerRPSController()
{
    float BL = BRPropeller.RPS + generateWhiteNoise(0.5);
    float BR = BRPropeller.RPS + generateWhiteNoise(0.5);
    float FL = BRPropeller.RPS + generateWhiteNoise(0.5);
    float FR = BRPropeller.RPS + generateWhiteNoise(0.5);
    return std::tuple<float, float, float, float>(BL, BR, FL, FR);
}

float HardDrone::propellerMomentZ(const Propeller& propeller)
{
    float RPS = propeller.RPS;
    float D5 = propeller.D4 * propeller.D;
    float Cq = propeller.Cq;

    return Cq * rho * RPS * RPS * D5; 
}

Vector3f HardDrone::totalPropellerMomentZ()
{
    Vector3f moment1 = propellerMomentZ(BLPropeller) * Vector3f{0,0,1};
    Vector3f moment2 = propellerMomentZ(BRPropeller) * Vector3f{0,0,-1};
    Vector3f moment3 = propellerMomentZ(FLPropeller) * Vector3f{0,0,-1};
    Vector3f moment4 = propellerMomentZ(FRPropeller) * Vector3f{0,0,1};

    return moment1 + moment2 + moment3 + moment4;
}

std::tuple<Propeller, Propeller, Propeller, Propeller> HardDrone::getPropellers()
{
    auto ret = std::make_tuple(BLPropeller, BRPropeller, FLPropeller, FRPropeller);
    return ret;
}

void HardDrone::setFourPropellerAcc(std::tuple<float, float, float, float> _acc)
{
    float BLacc = std::get<0>(_acc) + generateWhiteNoise();
    float BRacc = std::get<1>(_acc) + generateWhiteNoise();
    float FLacc = std::get<2>(_acc) + generateWhiteNoise();
    float FRacc = std::get<3>(_acc) + generateWhiteNoise();
}

void HardDrone::setFourPropellerRPS(std::tuple<float, float, float, float> _RPS)
{
    float BLrps = std::get<0>(_RPS) + generateWhiteNoise();
    float BRrps = std::get<1>(_RPS) + generateWhiteNoise();
    float FLrps = std::get<2>(_RPS) + generateWhiteNoise();
    float FRrps = std::get<3>(_RPS) + generateWhiteNoise();

    BLrps = std::min(BLrps, BLPropeller.maxRPS);
    BRrps = std::min(BRrps, BLPropeller.maxRPS);
    FLrps = std::min(FLrps, BLPropeller.maxRPS);
    FRrps = std::min(FRrps, BLPropeller.maxRPS);

    BLPropeller.RPS = BLrps;
    BRPropeller.RPS = BRrps;
    FLPropeller.RPS = FLrps;
    FRPropeller.RPS = FRrps;
}

float HardDrone::getPropellerD4() const
{
    return BLPropeller.D4;
}
