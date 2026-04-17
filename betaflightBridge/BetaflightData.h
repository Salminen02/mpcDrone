#pragma once
#include <memory.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>
#include "betaflight.h"
#define UNDEFINED_PWM_PULSE	900
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket close

class CBetaflightData
{
	int server_socket, client_socket;
	struct sockaddr_in server_addr, client_addr;
	bool isConnected;
public:
	fdm_packet bfOutFlightState;
	rc_packet bfOutCommandRC;

	servo_packet bfInMotors;
	servo_packet_raw bfInServos;

	CBetaflightData();
	bool Initialize();
	void Terminate();
	void Restart();
	bool SendData();
	bool RecvData();

	void setArm(bool on);
	void setAcroMode();
	void setMode(std::string modeName);
	void setThrottle(double throttle);
	void setStickPitch(double stickPitch);
	void setStickRoll(double stickRoll);
	void setStickYaw(double stickYaw);
	~CBetaflightData(void);
};

