#pragma once
#include <Eigen/Dense>
#include <memory>
#include "../drone/hardDrone.hpp"
#include "../drone/controller.hpp"
#include "iostream"


struct Time{
  float milliSeconds = 0;
  float seconds = 0;
  float minutes = 0;
};

struct DroneWhole{
  std::unique_ptr<HardDrone> hardDrone;
  std::unique_ptr<Controller> controller;
};

struct DroneWholeSnapshot{
  Eigen::Vector3f position;
  Eigen::Vector3f velocity;
  Eigen::Quaternionf orientation;
  std::tuple<Propeller, Propeller, Propeller, Propeller> fourPropellers;
  DroneBodyFrame bodyFrame;
  std::vector<PredictedState> predictedTrajectory;
};

struct SimSnapshot {
    std::vector<DroneWholeSnapshot> droneWholeSnapshots;
};


class Sim{
 public:
  Sim() = default;
  void updateSimulation(float dt);

  void pauseOn(){paused = true;};
  void pauseOff(){paused = false;};

  SimSnapshot getSnapshot() const;

  void startSimulation();

  void endSimulation();

  void resetSimulation();

  void updateTime(float dt);

  void setGoal(Vector3f xyz){droneGoalxyz = xyz;};

  Vector3f getGoal() const { return droneGoalxyz; };
  
  // Get controller for UI access
  Controller* getController() const {
    if (!droneWholes.empty() && droneWholes[0]) {
      return droneWholes[0]->controller.get();
    }
    return nullptr;
  }


 private:
    // Congig
  std::vector<std::unique_ptr<DroneWhole>> droneWholes{};
  Eigen::VectorXd q = (Eigen::VectorXd(12) << 2, 0.1, 2, 0.1, 2, 2, 2, 2, 2, 2, 4, 2).finished();
  Eigen::Vector4d r{0.1, 0.1, 0.1, 0.2};
  bool paused = false;
  bool ranOnce = false;
  Eigen::Vector3f droneGoalxyz{0,0,0};
  Time time;  
  float count = 0;
};