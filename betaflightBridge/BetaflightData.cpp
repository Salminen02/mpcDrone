#include "BetaflightData.h"


CBetaflightData::CBetaflightData(): server_socket(INVALID_SOCKET), client_socket(INVALID_SOCKET), isConnected(false)
{
    memset(&bfOutFlightState, 0, sizeof(bfOutFlightState));
    memset(&bfOutCommandRC, 0, sizeof(bfOutCommandRC));
    memset(&bfInMotors, 0, sizeof(bfInMotors));
    memset(&bfInServos, 0, sizeof(bfInServos));
    memset(&server_addr, 0, sizeof(server_addr));
    memset(&client_addr, 0, sizeof(client_addr));
    
    bfOutFlightState.imu_orientation_quat[0] = 1.0;
    bfOutFlightState.imu_linear_acceleration_xyz[2] = 9.81;
    bfOutFlightState.pressure = 101325.0;

    for (int ch = 0; ch < SIMULATOR_MAX_RC_CHANNELS; ch++)
        bfOutCommandRC.channels[ch] = UNDEFINED_PWM_PULSE;
}

CBetaflightData::~CBetaflightData(void)
{
    Terminate();
}

bool CBetaflightData::Initialize()
{
    // Create TCP server socket
    server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket == INVALID_SOCKET)
    {
        perror("socket creation failed");
        return false;
    }

    // Allow address reuse
    int opt = 1;
    if (setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1)
    {
        perror("setsockopt failed");
        closesocket(server_socket);
        return false;
    }

    // Set up server address
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(5761);  // Use different port from SITL

    // Bind socket
    if (bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR)
    {
        perror("bind failed");
        closesocket(server_socket);
        return false;
    }

    // Listen for connections
    if (listen(server_socket, 1) == SOCKET_ERROR)
    {
        closesocket(server_socket);
        return false;
    }

    isConnected = false;
    return true;
}

void CBetaflightData::Terminate()
{
    if (client_socket != INVALID_SOCKET)
        closesocket(client_socket);
    if (server_socket != INVALID_SOCKET)
        closesocket(server_socket);
}

void CBetaflightData::Restart()
{
    Terminate();
    Initialize();
}


bool CBetaflightData::SendData()
{
    // Accept new connections if not connected
    if (!isConnected)
    {
        socklen_t addr_len = sizeof(client_addr);
        client_socket = accept(server_socket, (struct sockaddr*)&client_addr, &addr_len);
        if (client_socket != INVALID_SOCKET)
        {
            isConnected = true;
            // Set socket to non-blocking
            int flags = fcntl(client_socket, F_GETFL, 0);
            fcntl(client_socket, F_SETFL, flags | O_NONBLOCK);
        }
    }

    if (isConnected) {
        // For now, just keep connection alive
        // MSP protocol implementation would go here
        return true;
    } 
    
    return false;
}

bool CBetaflightData::RecvData()
{  
    if (!isConnected) return false;

    char buf[1024];
    int res = recv(client_socket, buf, sizeof(buf), 0);
    
    if (res > 0)
    {
        // Basic MSP message handling would go here
        // For now, just indicate we received data
        return true;
    }
    else if (res == 0)
    {
        // Connection closed
        closesocket(client_socket);
        client_socket = INVALID_SOCKET;
        isConnected = false;
    }
    
    return false;
}

void CBetaflightData::setArm(bool on)
{
    // Oikea Betaflight armaus: AUX1 (channel 4)
    // Alkuperäisessä XML: arm-arvo 1000 = armed, 2000 = disarmed
    bfOutCommandRC.channels[4] = on ? 1000 : 2000; 
}

void CBetaflightData::setAcroMode()
{
    // Clear AUX channels except arm
    for (int i = 5; i < SIMULATOR_MAX_RC_CHANNELS; i++)
    {
        bfOutCommandRC.channels[i] = UNDEFINED_PWM_PULSE;
    }
}

void CBetaflightData::setMode(std::string modeName)
{
    setAcroMode();
    
    // Hardcoded mode mappings
    if (modeName == "manual") {
        bfOutCommandRC.channels[5] = 1000; // AUX2 low
    } else if (modeName == "acro") {
        bfOutCommandRC.channels[5] = 1500; // AUX2 mid
    } else if (modeName == "angle") {
        bfOutCommandRC.channels[5] = 2000; // AUX2 high
    } else if (modeName == "airplane") {
        bfOutCommandRC.channels[6] = 2000; // AUX3 high
    }
}

void CBetaflightData::setThrottle(double throttle)
{
    // Hardcoded: Channel 0 (throttle)
    bfOutCommandRC.channels[2] = (uint16_t) (PWM_MIN + (PWM_MAX - PWM_MIN) * throttle);
}

void CBetaflightData::setStickPitch(double stickPitch)
{
    // Hardcoded: Channel 1 (pitch)
    bfOutCommandRC.channels[1] = (uint16_t) (PWM_MIN + 0.5 * (PWM_MAX - PWM_MIN)  * (stickPitch + 1.0));
}

void CBetaflightData::setStickRoll(double stickRoll)
{
    // Hardcoded: Channel 2 (roll)
    bfOutCommandRC.channels[0] = (uint16_t) (PWM_MIN + 0.5 * (PWM_MAX - PWM_MIN)  * (stickRoll + 1.0));
}

void CBetaflightData::setStickYaw(double stickYaw)
{
    // Hardcoded: Channel 3 (yaw)
    bfOutCommandRC.channels[3] = (uint16_t) (PWM_MIN + 0.5 * (PWM_MAX - PWM_MIN)  * (stickYaw + 1.0));
}
