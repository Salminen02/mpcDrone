# Betaflight Bridge

A C++ bridge for communicating with Betaflight SITL (Software In The Loop) simulator via UDP.

## Features

- **UDP Communication**: Direct communication with Betaflight SITL on ports 9003 (FDM data) and 9004 (RC commands)
- **Flight Data**: Sends IMU orientation (quaternions), accelerometer, and barometer data
- **RC Control**: Sends throttle, roll, pitch, yaw, and arming commands
- **Motor Output**: Receives motor PWM outputs from Betaflight (port 9001)
- **Real-time Simulation**: Updates at 1kHz for smooth flight simulation

## Files

- `main.cpp` - Main simulation loop with UDP communication
- `BetaflightData.h/cpp` - Betaflight data structures and RC control methods
- `betaflight.h` - Betaflight SITL protocol definitions (FDM, RC, servo packets)

## Building

```bash
g++ main.cpp BetaflightData.cpp -o betaflight_bridge -lm -pthread
```

## Usage

1. Start Betaflight SITL simulator
2. Run the bridge:
   ```bash
   ./betaflight_bridge
   ```
3. The bridge will:
   - Send flight attitude data (rolling motion)
   - Send RC commands (throttle, arming)
   - Listen for motor outputs from Betaflight

## Protocol

### Outgoing to Betaflight SITL:
- **Port 9003**: `fdm_packet` - Flight dynamics (IMU, position, pressure)
- **Port 9004**: `rc_packet` - RC commands (16 channels, PWM 1000-2000)

### Incoming from Betaflight SITL:
- **Port 9001**: `servo_packet_raw` - Motor PWM outputs

## Integration

This bridge can be integrated with:
- Flight simulators (X-Plane, FlightGear, custom simulators)
- Physics engines
- Hardware-in-the-loop setups
- Drone simulation frameworks

## Example RC Channel Mapping

- Channel 0: Throttle (TAER order)
- Channel 1: Roll/Aileron
- Channel 2: Pitch/Elevator  
- Channel 3: Yaw/Rudder
- Channel 4: ARM switch (1000=armed, 2000=disarmed)

## Dependencies

- Standard C++ libraries
- POSIX sockets (Linux/macOS)
- Math library (-lm)