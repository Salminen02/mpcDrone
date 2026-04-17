#include "simThread.hpp"

SimThread::SimThread(std::unique_ptr<Sim> _sim):
    sim(std::move(_sim)),
    running(false)
{}

SimSnapshot SimThread::getSimSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex);
    return sim->getSnapshot();  // Kopioi datan turvallisesti
}

void SimThread::threadLoop()
{
    while (running.load()) {  // . eikä -> (running on arvo, ei pointteri)
        auto startTime = std::chrono::high_resolution_clock::now();
        
        // Lock and update simulation
        {
            std::lock_guard<std::mutex> lock(mutex);  // Vain mutex, ei state->mutex
            
            // Only update if not paused
            if (!paused) {
                sim->updateSimulation(1.0f / updateFreq * timeScale);  // Käytä timeScalea
            }
        }
        
        // Sleep to maintain update frequency
        auto endTime = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        auto sleepTime = std::chrono::milliseconds(static_cast<int>(1000.0f / updateFreq)) - elapsed;
        
        if (sleepTime.count() > 0) {
            std::this_thread::sleep_for(sleepTime);
        }
    }
}

void SimThread::start() {
    if (!running.load()) {
        running.store(true);
        sim->startSimulation();  // Alusta simulaatio
        thread = std::thread(&SimThread::threadLoop, this);  // Käynnistä thread
    }
}

void SimThread::stop() {
    running.store(false);  // Signaali threadille lopettaa
    if (thread.joinable()) {
        thread.join();  // Odota loppuun
    }
}

void SimThread::pauseButton() {
    bool expected = paused.load();
    while (!paused.compare_exchange_weak(expected, !expected)) {}
}

void SimThread::resetSimulation()
{
    std::lock_guard<std::mutex> lock(mutex);
    sim->resetSimulation();
    paused = true;
}
