#include "sim.hpp"
#include <memory>


// Ensure the correct namespace is used for ControllerDevice everywhere
 // Remove or adjust this if not needed, or qualify ControllerDevice with its namespace explicitly


void Sim::updateSimulation(float dt)
{
    for (auto& droneWhole : droneWholes){
            droneWhole->hardDrone->move(dt);
            //droneWhole->controller->control(dt);
            droneWhole->controller->control(dt);
            //droneWhole->controller->setGoalX(droneGoalxyz);
            if (auto* nmpc = dynamic_cast<acadosNMPC*>(droneWhole->controller.get())) {
                nmpc->setBallConstraint(droneGoalxyz, droneGoalRadius);
            }
    }
    updateTime(dt);
}

SimSnapshot Sim::getSnapshot() const
{
    SimSnapshot simSnapshot;

    for (auto& droneWholePtr : droneWholes){
        DroneWhole* droneWhole = droneWholePtr.get();
        HardDrone* hardDrone = droneWhole->hardDrone.get();
        Controller* controller = droneWhole->controller.get();
        DroneWholeSnapshot droneWholeSnapshot;

        droneWholeSnapshot.position = hardDrone->getPosition();
        droneWholeSnapshot.bodyFrame = hardDrone->getBodyFrame();
        droneWholeSnapshot.velocity = hardDrone->getVelocity();
        droneWholeSnapshot.orientation = hardDrone->getOrientationQ();

        // Ennustettu reitti kontrollerilta
        if (auto* nmpc = dynamic_cast<acadosNMPC*>(controller)) {
            droneWholeSnapshot.predictedTrajectory = nmpc->getPredictedTrajectory();
            droneWholeSnapshot.costBreakdown       = nmpc->getCostBreakdown();
        }
    
        simSnapshot.droneWholeSnapshots.push_back(droneWholeSnapshot);
        }

    return simSnapshot;
};

void Sim::startSimulation()
{
    auto droneWhole = std::make_unique<DroneWhole>();
    auto hardDrone = std::make_unique<HardDrone>();

    Eigen::Vector4d r2 = r.array();
    Eigen::VectorXd q2 = q.array() + count * (Eigen::ArrayXd(12) << 0, 0.13, 0, 0.13, 0, 0.13, 0, 0, 0, 0, 0, 0).finished();
    count++;
    auto controller = std::make_unique<acadosNMPC>(hardDrone.get());
    //controller->setControllerDevice(controllerDevice);
    droneWhole->hardDrone = std::move(hardDrone);
    droneWhole->controller = std::move(controller);
    droneWholes.push_back(std::move(droneWhole));
}

void Sim::endSimulation()
{
}

void Sim::resetSimulation()
{
    endSimulation();
    startSimulation();
}

void Sim::updateTime(float dt)
{
    time.milliSeconds += 1000*dt;
    if (time.milliSeconds > 1000){
        time.milliSeconds -= 1000;
        time.seconds += 1;
    }
    if (time.seconds > 60){
        time.seconds -= 60;
        time.minutes += 1;
    }

}
