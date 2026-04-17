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

    nlp_solver = quadrotor_mpcc_acados_get_nlp_solver(capsule);
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

        const double clEndBoost = 30; // 0.0 = no boost, larger = stronger terminal weighting
        const double denomN = (N > 0) ? static_cast<double>(N) : 1.0;

        // Pallon sijainti yaw-normalisoituun kehykseen
        Eigen::Vector3f ballLocal = inv_yaw_q * ballCenter;

        for (int k = 0; k <= N; ++k) {
            const double alpha = static_cast<double>(k) / denomN;
            const double clStage = Cl * (1.0 + clEndBoost * alpha * alpha * alpha);
            double p[7] = {(double)yaw, mu, clStage,
                           (double)ballLocal.x(), (double)ballLocal.y(), (double)ballLocal.z(),
                           (double)ballRadius};
            quadrotor_mpcc_acados_update_params(capsule, k, p, 7);
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

            // Cost breakdown: replicate each term of the stage cost over the horizon.
            // circle_path in yaw-normalized frame (mirrors Acados.py circle_path):
            //   p_ref = [cy*a*sin(θ)+sy*b*cos(θ), -sy*a*sin(θ)+cy*b*cos(θ), 0]
            //   t_ref = normalise([cy*a*cos(θ)-sy*b*sin(θ), -sy*a*cos(θ)-cy*b*sin(θ), 0])
            const double a_path = 5.0, b_path = 10.0;
            const double Cc = 10.0, Cv = 0.75, Cr = 5.0;
            const double W_ball_quad = 5000.0;
            const double cy = std::cos((double)yaw), sy = std::sin((double)yaw);

            CostBreakdown cb{};
            for (int k = 0; k < N; ++k) {
                double xk[13], uk[5];
                ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, k, "x", xk);
                ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, k, "u", uk);

                const double theta_k = xk[12];
                const double px_w =  a_path * std::sin(theta_k);
                const double py_w =  b_path * std::cos(theta_k);
                const double tx_w =  a_path * std::cos(theta_k);
                const double ty_w = -b_path * std::sin(theta_k);

                const double px_ref =  cy * px_w + sy * py_w;
                const double py_ref = -sy * px_w + cy * py_w;
                const double tx_r   =  cy * tx_w + sy * ty_w;
                const double ty_r   = -sy * tx_w + cy * ty_w;
                const double tnorm  = std::sqrt(tx_r*tx_r + ty_r*ty_r);
                const double tnx = tx_r / tnorm, tny = ty_r / tnorm;

                const double ex = xk[0] - px_ref;
                const double ey = xk[1] - py_ref;
                const double ez = xk[2];  // pz_ref = 0
                const double lag   = tnx * ex + tny * ey;
                const double err2  = ex*ex + ey*ey + ez*ez;
                const double contour = err2 - lag * lag;

                const double alpha   = (double)k / (N > 0 ? N : 1);
                const double clStage = Cl * (1.0 + 30.0 * alpha*alpha*alpha);

                cb.lagCost      += clStage * lag * lag;
                cb.contourCost  += Cc * contour;
                cb.progressCost += -mu * uk[4];
                cb.linVelCost   += Cv * (xk[3]*xk[3] + xk[4]*xk[4] + xk[5]*xk[5]);
                cb.angVelCost   += Cr * (xk[9]*xk[9] + xk[10]*xk[10] + xk[11]*xk[11]);

                double sl[1] = {0.0};
                ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, k, "sl", sl);
                cb.obstacleCost += W_ball_quad * sl[0] * sl[0];
            }
            // Terminal stage: only obstacle slack (terminal cost expr = 0)
            {
                double sl_e[1] = {0.0};
                ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, N, "sl", sl_e);
                cb.obstacleCost += W_ball_quad * sl_e[0] * sl_e[0];
            }
            lastCostBreakdown = cb;
        }
    }
    if (counter > 300){
        mixer(lastU);
        Cl = 1000;
    }
    else{
        PID(dt);
    }
    if (counter > 200){
        mu = 150;
        Cl = 0.1;
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
