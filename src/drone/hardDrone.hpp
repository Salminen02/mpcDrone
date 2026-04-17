#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility>
#include <iostream>
#include <cmath>

using Eigen::Vector3f;

struct Propeller{
    float RPS = 0.0f;
    float acc = 0.0f;
    const float D = 0.25f;  // ANNA OLETUSARVO!
    float D4 = D * D * D * D;
    Eigen::Vector2f coord{0.0f, 0.0f};
    float Ct = 0.01f;  // ANNA OLETUSARVO!
    float Cq = 0.01f;
    float maxRPS = 200;
    
    Propeller(Eigen::Vector2f _coord = {0,0}) : coord(_coord) {
        D4 = D * D * D * D;
    }
};

struct DroneBodyFrame{
    Vector3f X_forward; // Paikallinen X-akseli Maailmassa
    Vector3f Y_right;      // Paikallinen Y-akseli Maailmassa
    Vector3f Z_up;   // Paikallinen Z-akseli Maailmassa
};

class HardDrone{
 public:
    HardDrone();
    void setOrientation(float yaw, float pitch, float roll);
    float propellerForce(const Propeller& propeller);
    float propellerForces();

    Vector3f propellerMomentLocal(const Propeller& propeller);
    Vector3f propellerMomentsLocal();
    void setOrientationQ_zero();
    void move(float dt);
    DroneBodyFrame getBodyFrame();
    Eigen::Quaternionf getOrientationQ(){ return orientationQ; };

    Eigen::Quaternionf getOrienatationQcontroller();

    Vector3f getPosition() const { return position;};
    Vector3f getVelocity() const { return vel; };
    DroneBodyFrame getBodyFrame() const { return bodyFrame; };
    Vector3f getAngularVelocityLocal() {
        return orientationQ.inverse() * angularVel;  // Quaternion * Vector3f
    }
    Vector3f getAngularVelocityWorld(){
        return angularVel;
    }

            //  BL     BR     Fl     FR
    std::tuple<float, float, float, float> getPropellerRPSController();

    float propellerMomentZ(const Propeller& propeller);
    Vector3f totalPropellerMomentZ();

    std::tuple<Propeller, Propeller, Propeller, Propeller> getPropellers();

    void setPropellerAcc(Propeller& propeller, float value);
    void setFourPropellerAcc(std::tuple<float, float, float, float> _acc);

    void setPropellerRPS(Propeller& propeller, float value);
    void setFourPropellerRPS(std::tuple<float, float, float, float> _acc);

    float getRho() const {return rho;};
    float getMass() const { return m; };
    float getPropellerD4() const;
    float getCt() const {return BLPropeller.Ct;};

    Vector3f getAngularAcc() const { return angularAcc; };

    Vector3f getVel() { return vel; };

    float getR() {return std::sqrt(BLPropeller.coord.x()*BLPropeller.coord.x() + BLPropeller.coord.y()*BLPropeller.coord.y());};

    float getCq() { return BLPropeller.Cq; };
    float getCt() { return BLPropeller.Ct; };
    float getD() { return BLPropeller.D; };
    Eigen::Vector3d getI() { return I; };
 private:
    float squarePropellerDrone = 0.3f;  // MÄÄRITTELE ENNEN propellereitä!
    
    Propeller BLPropeller;
    Propeller BRPropeller;
    Propeller FLPropeller;
    Propeller FRPropeller;
    
    float m = 0.5f;

    Vector3f position{0,10,0};
    Vector3f vel{0,0,0};
    Vector3f acc{0,0,0};
    Vector3f angularVel{0, 0, 0};
    Vector3f angularAcc{0, 0, 0};
    Vector3f moment{0, 0, 0};

    Eigen::Quaternionf orientationQ;

    float angularInertia = 0.05;
    Eigen::Vector3d I{0.0075, 0.0075, 0.03};

    float rho = 1.2;

    DroneBodyFrame bodyFrame; 
};