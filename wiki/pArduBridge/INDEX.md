# pArduBridge Documentation Index

Welcome to the pArduBridge documentation! This directory contains comprehensive information about the pArduBridge application, which bridges MOOS-IVP autonomy system with ArduPilot autopilot software for UAV operations.

---

## Documentation Files

### 📘 [README.md](README.md)
**Main documentation and reference guide**

Complete reference documentation including:
- Overview and purpose
- Configuration parameters and examples
- MOOS variable subscriptions and publications
- Usage instructions
- Safety considerations
- Troubleshooting guide

**Start here for**: Configuration, MOOS variables, general usage

---

### 🏗️ [ARCHITECTURE.md](ARCHITECTURE.md)
**Detailed architecture and design documentation**

In-depth technical documentation covering:
- System architecture diagrams
- Component descriptions (ArduBridge, UAV_Model, SetpointManager, WarningSystem)
- Threading model and synchronization
- Data flow diagrams (command flow, telemetry flow)
- State machine design
- Class relationships and diagrams
- Sequence diagrams
- Design patterns used
- Performance considerations

**Start here for**: Understanding internals, contributing, advanced debugging

---

### 🚀 [QUICKSTART.md](QUICKSTART.md)
**Quick start guide for new users**

Step-by-step setup guide including:
- Prerequisites checklist
- ArduPilot SITL setup
- Mission file configuration
- Build instructions
- Launch procedures
- Basic operations (arming, commanding, monitoring)
- Common issues and solutions
- Status monitoring techniques

**Start here for**: Getting up and running quickly, first-time setup

---

## Quick Navigation

### I want to...

**...get started quickly**
→ [QUICKSTART.md](QUICKSTART.md)

**...understand configuration options**
→ [README.md - Configuration](README.md#configuration)

**...see MOOS variable interfaces**
→ [README.md - MOOS Variables](README.md#moos-variables)

**...understand the architecture**
→ [ARCHITECTURE.md](ARCHITECTURE.md)

**...troubleshoot connection issues**
→ [QUICKSTART.md - Common Issues](QUICKSTART.md#common-issues-and-solutions)

**...understand threading and data flow**
→ [ARCHITECTURE.md - Threading Model](ARCHITECTURE.md#threading-model)

**...learn about state management**
→ [ARCHITECTURE.md - State Machine](ARCHITECTURE.md#state-machine)

**...see how commands are executed**
→ [ARCHITECTURE.md - Sequence Diagrams](ARCHITECTURE.md#sequence-diagrams)

---

## Key Concepts

### What is pArduBridge?
pArduBridge is a MOOS application that acts as a bridge between:
- **MOOS-IVP**: A powerful autonomy middleware for marine and aerial vehicles
- **ArduPilot**: Open-source autopilot software for UAVs

It translates high-level autonomy commands from MOOS into low-level MAVLink commands for ArduPilot and publishes telemetry back to the MOOS community.

### Main Components

1. **ArduBridge**: Main MOOS application class
2. **UAV_Model**: Manages MAVLink connection and commands
3. **SetpointManager**: Thread-safe storage for desired values
4. **WarningSystem**: Monitoring and warning management

### Threading Architecture

- **Main Thread**: MOOS loop, processes mail, manages state
- **Command Sender Thread**: Executes commands, sends setpoints, polls parameters

---

## Document Maintenance

These documents should be updated as pArduBridge evolves. Key areas that may require updates:

- New MAVLink commands or telemetry streams
- Additional autopilot helm modes
- Enhanced safety features
- Configuration parameter changes
- Performance optimizations
- Bug fixes affecting documented behavior

---

## Additional Resources

### External Documentation
- [MOOS-IVP Documentation](https://oceanai.mit.edu/moos-ivp/)
- [ArduPilot Documentation](https://ardupilot.org/)
- [MAVSDK Documentation](https://mavsdk.mavlink.io/)
- [MAVLink Protocol](https://mavlink.io/)

### Source Code
- `src/pArduBridge/ArduBridge.h` and `ArduBridge.cpp` - Main application
- `src/pArduBridge/UAV_Model.h` and `UAV_Model.cpp` - UAV interface
- `src/pArduBridge/SetpointManager.h` - Setpoint management
- `src/pArduBridge/WarningSystem.h` - Warning system
- `src/pArduBridge/ArduBridge_Info.cpp` - Command-line help

### Example Missions
Look for example mission files in:
```
missions/
```

---

## Contributing

When contributing to pArduBridge, please:

1. Update relevant documentation files
2. Add new features to appropriate sections
3. Update diagrams if architecture changes
4. Test documentation examples
5. Maintain consistent formatting

---

## Credits

**Author**: Steve Carter Feujo Nomeny  
**Organization**: NTNU, MIT  
**Date**: September 9th, 2024  
**Documentation Created**: October 12th, 2024

---

## Feedback

If you find errors, unclear sections, or have suggestions for improving this documentation, please open an issue or submit a pull request at:

https://github.com/cbenjamin23/moos-ivp-uav-base

---

**Last Updated**: October 2024
