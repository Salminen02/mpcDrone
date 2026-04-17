#include "controller.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

Eigen::Vector3f quatToEul(Eigen::Quaternionf q);

acadosNMPC::acadosNMPC(HardDrone *_hardDrone)
: Controller(_hardDrone), timeSinceLastSolve(0.0f), currentTheta(-M_PI/4),
  lastU(Eigen::Vector4d::Constant(0.5 * 9.81 / 4.0)), capsule(nullptr)
{
    capsule = quadrotor_mpcc_acados_create_capsule();
    quadrotor_mpcc_acados_create(capsule);

    nlp_config = quadrotor_mpcc_acados_get_nlp_config(capsule);
    nlp_dims = quadrotor_mpcc_acados_get_nlp_dims(capsule);
    nlp_out = quadrotor_mpcc_acados_get_nlp_out(capsule);
    nlp_in = quadrotor_mpcc_acados_get_nlp_in(capsule);
    N = nlp_dims->N;

    const double hover = 0.5 * 9.81 / 4.0;
    double u0[5] = {hover, hover, hover, hover, 0.0};
    double x0[13] = {};
    x0[2] = 1.0;
    for (int k = 0; k <= N; ++k)
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, k, "x", x0);
    for (int k = 0; k < N; ++k)
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, k, "u", u0);
}

acadosNMPC::~acadosNMPC()
{
    if (capsule) {
        quadrotor_mpcc_acados_free(capsule);
        quadrotor_mpcc_acados_free_capsule(capsule);
    }
}

void acadosNMPC::control(float dt)
{
    Eigen::Quaternionf q = hardDrone->getOrientationQ();
    Vector3f eulerAngles = quatToEul(q);
    float yaw = eulerAngles.z();
    Eigen::Quaternionf yaw_q(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    Eigen::Quaternionf inv_yaw_q = yaw_q.inverse();
    Vector3f pos = inv_yaw_q * hardDrone->getPosition();
    Vector3f vel = inv_yaw_q * hardDrone->getVelocity();
    Vector3f angles = quatToEul(yaw_q.inverse() * q);
    Vector3f angVels = inv_yaw_q * hardDrone->getAngularVelocityWorld();

    double x[13] = {
        pos.x(), pos.y(), pos.z(),
        vel.x(), vel.y(), vel.z(),
        angles.x(), angles.y(), 0.0,
        angVels.x(), angVels.y(), angVels.z(),
        currentTheta
    };

    timeSinceLastSolve += dt;
    if (timeSinceLastSolve >= 0.015f) {
        timeSinceLastSolve = 0.0f;

        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x);

        const double clEndBoost = 5.0; // 0.0 = no boost, larger = stronger terminal weighting
        const double denomN = (N > 0) ? static_cast<double>(N) : 1.0;
        for (int k = 0; k <= N; ++k) {
            const double alpha = static_cast<double>(k) / denomN;
            const double clStage = Cl * (1.0 + clEndBoost * alpha * alpha * alpha);
            double p[3] = {(double)yaw, mu, clStage};
            quadrotor_mpcc_acados_update_params(capsule, k, p, 3);
        }

        int status = quadrotor_mpcc_acados_solve(capsule);
        if (status != 0) {
            std::cout << "Acados solver FAILED, status=" << status << std::endl;
        } else {
            double u_opt[5];
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u_opt);
            lastU(0) = u_opt[0];
            lastU(1) = u_opt[1];
            lastU(2) = u_opt[2];
            lastU(3) = u_opt[3];

            double x1[13];
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", x1);
            currentTheta = x1[12];

            // Tallennetaan ennustettu reitti
            Eigen::Quaternionf yawQ_inv = yaw_q.inverse();
            predictedTrajectory.clear();
            predictedTrajectory.reserve(N + 1);
            for (int k = 0; k <= N; ++k) {
                double xk[13];
                ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, k, "x", xk);
                // Sijainti on yaw-normalisoitu → käännetään takaisin maailmakoordinaatteihin
                Eigen::Vector3f posLocal((float)xk[0], (float)xk[1], (float)xk[2]);
                Eigen::Vector3f posWorld = yaw_q * posLocal;
                // Orientaatio: Euler rungossa + yaw lisätään takaisin
                Eigen::Quaternionf tiltK =
                    Eigen::AngleAxisf((float)xk[6], Eigen::Vector3f::UnitX()) *
                    Eigen::AngleAxisf((float)xk[7], Eigen::Vector3f::UnitY());
                Eigen::Quaternionf orientWorld = yaw_q * tiltK;
                predictedTrajectory.push_back({posWorld, orientWorld});
            }
        }
    }
    if (counter > 100){
        mixer(lastU);
        Cl = 1000;
    }
    else{
        PID(dt);
    }
    if (counter > 200){
        mu = 50;
        Cl = 1;
    }
    counter++;
}

void acadosNMPC::mixer(Eigen::Vector4d u)
{
    float fl = u(0);
    float bl = u(1);
    float br = u(2);
    float fr = u(3);

    float denumerator = hardDrone->getRho() * hardDrone->getPropellerD4() * hardDrone->getCt();

    float FLRps = sqrt(std::abs(fl / denumerator));
    float FRRps = sqrt(std::abs(fr / denumerator));
    float BLRps = sqrt(std::abs(bl / denumerator));
    float BRRps = sqrt(std::abs(br / denumerator));

    hardDrone->setFourPropellerRPS(std::make_tuple(BLRps, BRRps, FLRps, FRRps));
}

Eigen::Vector3f quatToEul(Eigen::Quaternionf q)
{
    q.normalize();
    float roll = std::atan2(2.0f * (q.w() * q.x() + q.y() * q.z()),
                            1.0f - 2.0f * (q.x() * q.x() + q.y() * q.y()));
    float sinp = 2.0f * (q.w() * q.y() - q.z() * q.x());
    float pitch = (std::abs(sinp) >= 1.0f) ? std::copysign(1.570796f, sinp) : std::asin(sinp);
    float yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()),
                           1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
    return Eigen::Vector3f(roll, pitch, yaw);
}

void Controller::PID(float dt)
{
    // --- State (body-frame tilt by removing yaw first) ---
    Eigen::Quaternionf q = hardDrone->getOrientationQ();
    Eigen::Vector3f eulerFull = quatToEul(q);
    float yaw = eulerFull.z();
    Eigen::Quaternionf yawQ(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    Eigen::Quaternionf tiltQ = yawQ.inverse() * q;   // pure roll+pitch
    Eigen::Vector3f tiltEuler = quatToEul(tiltQ);
    float roll  = tiltEuler.x();
    float pitch = tiltEuler.y();

    Eigen::Vector3f w = hardDrone->getAngularVelocityLocal();

    // --- Hover thrust ---
    float T = hardDrone->getMass() * 9.81f;  // total thrust = weight

    // --- PD gains ---
    float Kp = 0.2f;
    float Kd = 0.01f;

    // --- Desired torques (negative feedback) ---
    //  τ_x = y*Fz  summed over props  = L*(-F_BL - F_BR + F_FL + F_FR)
    //  τ_y = -x*Fz summed over props  = L*( F_BL - F_BR + F_FL - F_FR)
    float tau_x = -Kp * roll  - Kd * w.x();
    float tau_y = -Kp * pitch - Kd * w.y();

    // --- Mixing (invert the torque equations) ---
    float L = 0.3f;  // squarePropellerDrone
    float inv4L = 1.0f / (4.0f * L);
    float T4    = T / 4.0f;

    float F_BL = T4 - tau_x * inv4L + tau_y * inv4L;
    float F_BR = T4 - tau_x * inv4L - tau_y * inv4L;
    float F_FL = T4 + tau_x * inv4L + tau_y * inv4L;
    float F_FR = T4 + tau_x * inv4L - tau_y * inv4L;
 
    // --- Force → RPS:  F = ρ · D⁴ · Ct · RPS² ---
    float denom = hardDrone->getRho() * hardDrone->getPropellerD4() * hardDrone->getCt();

    float BLRps = sqrt(std::max(0.0f, F_BL/denom));
    float BRRps = sqrt(std::max(0.0f, F_BR/denom));
    float FLRps = sqrt(std::max(0.0f, F_FL/denom));
    float FRRps = sqrt(std::max(0.0f, F_FR/denom));

    auto RPSs = std::make_tuple(BLRps, BRRps, FLRps, FRRps);
    hardDrone->setFourPropellerRPS(RPSs);
}
