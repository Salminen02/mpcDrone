#pragma once
#include "hardDrone.hpp"
#include <Eigen/Dense>
#include <vector>
#include "c_generated_code/acados_solver_quadrotor_mpcc.h"

struct PredictedState {
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
};

class Controller {
 public:
  Controller(HardDrone* _hardDrone) : hardDrone(_hardDrone) {};
  virtual void control(float dt) = 0;
  void PID(float dt);

 protected:
  HardDrone* hardDrone;

};

// Forward declare to avoid heavy acados headers in hpp
struct quadrotor_mpcc_solver_capsule;

class acadosNMPC : public Controller {
 public:
   acadosNMPC(HardDrone* _hardDrone);
   ~acadosNMPC();
   void control(float dt) override;
   void mixer(Eigen::Vector4d u);

   double mu = 5;
   double Cl = 5;

   void setBallConstraint(Eigen::Vector3f center, float radius) {
       ballCenter = center;
       ballRadius = radius;
   }

   double getLastCost() const { return lastCost; }

   const std::vector<PredictedState>& getPredictedTrajectory() const { return predictedTrajectory; }
 private:
   Eigen::Vector3f ballCenter{0.0f, 0.0f, 5.0f};
   float ballRadius = 0.01f;   double lastCost = 0.0;   int counter = 0;
   int N;
   float timeSinceLastSolve;
   double currentTheta;
   Eigen::Vector4d lastU;
   quadrotor_mpcc_solver_capsule* capsule;
   ocp_nlp_config *nlp_config;
   ocp_nlp_dims *nlp_dims;
   ocp_nlp_in *nlp_in;
   ocp_nlp_out *nlp_out;
   std::vector<PredictedState> predictedTrajectory;

};