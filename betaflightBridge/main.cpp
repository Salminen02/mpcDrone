#include "BetaflightData.h"
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <fcntl.h>

void setQuatFromRPY(double roll, double pitch, double yaw, double* q) {
    double cy = cos(yaw * 0.5); double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5); double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5); double sr = sin(roll * 0.5);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

int main() {
    CBetaflightData* betaflight = new CBetaflightData();
    betaflight->setArm(false);
    // Setup UDP sockets for SITL communication
    int send_socket = socket(AF_INET, SOCK_DGRAM, 0);
    int recv_socket = socket(AF_INET, SOCK_DGRAM, 0);
    
    struct sockaddr_in fdm_addr = {AF_INET, htons(9003), inet_addr("127.0.0.1")};
    struct sockaddr_in rc_addr = {AF_INET, htons(9004), inet_addr("127.0.0.1")};
    
    // Bind receive socket to listen for motor outputs from Betaflight
    struct sockaddr_in motor_listen = {AF_INET, htons(9001), INADDR_ANY};
    bind(recv_socket, (struct sockaddr*)&motor_listen, sizeof(motor_listen));
    
    // Set receive socket non-blocking
    int flags = fcntl(recv_socket, F_GETFL, 0);
    fcntl(recv_socket, F_SETFL, flags | O_NONBLOCK);
    
    double t_sec = 0.0;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;

    int iter = 0;

    std::cout << "Käytetään Betaflight luokkia UDP-kommunikaatioon..." << std::endl;
    std::cout << "Kuunnellaan motor outputtia portissa 9001..." << std::endl;

    bool stop = true;
    std::cout << "Stop mode (1=true, 0=false)? ";
    std::cin >> stop;

    while (true) {
        // VASTAANOTA motor outputti Betaflightilta
        char motor_buf[1024];
        struct sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);
        
        int bytes_received = recvfrom(recv_socket, motor_buf, sizeof(motor_buf), 0, 
                                     (struct sockaddr*)&from_addr, &from_len);
        /**
        if (bytes_received > 0 && iter % 1000 == 0) {
            // Tulkitse motor data (servo_packet_raw struktuuri)

            if (bytes_received >= sizeof(betaflight->bfInServos)) {
                memcpy(&betaflight->bfInServos, motor_buf, sizeof(betaflight->bfInServos));
            }
        }
        iter++;
        */

        // 1. ANTURIT - Käytä Betaflight struktureja
        memset(&betaflight->bfOutFlightState, 0, sizeof(betaflight->bfOutFlightState));
        betaflight->bfOutFlightState.timestamp = t_sec;
        
        // Simuloi roll-liike
        if (stop){
            roll_deg = 0.0;}
        else{
            roll_deg = 3 * sin(t_sec);
            //pitch_deg = 0.5 * sin(t_sec);
        }
        //roll_deg = 0;
        double r = roll_deg * M_PI / 180.0;
        double p = pitch_deg * M_PI / 180.0;
        const double g = 1;
        betaflight->bfOutFlightState.imu_linear_acceleration_xyz[0] = g * sin(r);           // X: sivuttainen komponentti
        betaflight->bfOutFlightState.imu_linear_acceleration_xyz[1] = g * sin(p);           // Y: pitkittäinen komponentti
        betaflight->bfOutFlightState.imu_linear_acceleration_xyz[2] = g * cos(r) * cos(p); // Z: pystysuora (negatiivinen = alaspäin)
        
        //setQuatFromRPY(r, p, 0, betaflight->bfOutFlightState.imu_orientation_quat);
        
        // Gyrodata = kulmanopeus (derivaatta asennosta)
        if (!stop){
        betaflight->bfOutFlightState.imu_angular_velocity_rpy[0] = 3 * cos(t_sec) * M_PI / 180.0;  // roll rate
        betaflight->bfOutFlightState.imu_angular_velocity_rpy[1] = 0;  //-0.5 * cos(t_sec) * M_PI / 180.0;  // pitch rate
        betaflight->bfOutFlightState.imu_angular_velocity_rpy[2] = 0.0;  // yaw rate
        }
        
        //betaflight->bfOutFlightState.imu_linear_acceleration_xyz[2] = -9.81;
        betaflight->bfOutFlightState.pressure = 101325.0;
        
        // Lähetä FDM data
        int fdm_bytes = sendto(send_socket, &betaflight->bfOutFlightState, sizeof(betaflight->bfOutFlightState), 
               0, (struct sockaddr*)&fdm_addr, sizeof(fdm_addr));
        
        if (fdm_bytes < 0) {
            perror("FDM sendto failed");
        }

        // 2. OHJAUS - Käytä Betaflight RC metodeja
        if (int(t_sec * 1000) % 20 == 0) {
            // Käytä luokan metodeja RC-komentojen asettamiseen
            // Siniaalto throttle
            if (stop){
                betaflight->setArm(false);         // ARM drone
                betaflight->setThrottle(0.0);
                betaflight->setStickRoll(0.0);    // Roll
                betaflight->setStickPitch(0.0);   // Pitch
                betaflight->setStickYaw(0.0);
            }
            else{
                betaflight->setArm(true);
                betaflight->setThrottle(0.3);
                betaflight->setStickRoll(0.0);    // Roll
                betaflight->setStickPitch(0.0);   // Pitch
                betaflight->setStickYaw(0.0);
            }     // Yaw
            
            // Aseta timestamp RC-pakettiin
            betaflight->bfOutCommandRC.timestamp = t_sec;
            
            // Lähetä RC data
            int rc_bytes = sendto(send_socket, &betaflight->bfOutCommandRC, sizeof(betaflight->bfOutCommandRC), 
                   0, (struct sockaddr*)&rc_addr, sizeof(rc_addr));
                   
            if (rc_bytes < 0) {
                perror("RC sendto failed");
            }
        }

        t_sec += 0.001;
        usleep(1000); 
    }
    
    delete betaflight;
    close(send_socket);
    close(recv_socket);
    return 0;
}