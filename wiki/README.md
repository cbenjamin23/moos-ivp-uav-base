# Getting Started with MOOS-IVP UAV

This documentation provides guidance on setting up and running the MOOS-IVP UAV system.

## Quick Navigation

### Preliminary Payload Computer Setup
- **[Raspberry Pi MOOS-IvP Setup.md](Raspberry_Pi_MOOS-IvP_Setup.md)** - Preliminary setup if installing moos-ivp-uav-base on a raspberry pi

### Essential Setup Documentation
- **[Installation & Setup.md](Installation_&_Setup.md)** - Core system installation and setup instructions
  - Clone repository and submodules
  - Install MOOS-IVP
  - Install MOOS IVP SWARM Toolbox
  - Setup ArduPilot, Gazebo, and related tools

### Key Components
- **[MAVSDK Setup & Usage.md](MAVSDK_Setup_&_Usage.md)** - MAVSDK installation and building custom applications
- **[ArduPilot & MavProx.md](ArduPilot_&_MavProx.md)** - ArduPilot configuration, parameters, Mission Planner setup, and swarm capabilities
- **[System Launch Guide.md](System_Launch_Guide.md)** - How to launch missions and ground stations
- **[Gazebo Plugin Configuration.md](Gazebo_Plugin_Configuration.md)** - Gazebo plugin details and version information

### UAV Mission Documentation
- **[UAV Mission Configuration.md](UAV_Mission_Configuration.md)** - Complete guide to missionConfig.yaml parameters and structure
- **[UAV Mission ARDU Commands.md](UAV_Mission_ARDU_Commands.md)** - Ground Control Station commands and their usage in .moos files
- **[UAV Mission Launch Scripts.md](UAV_Mission_Launch_Scripts.md)** - Launch script structure and how they relate to YAML configuration

### Additional Resources
- **[Telemetry Data Extraction.md](Telemetry_Data_Extraction.md)** - Managing mission data and telemetry extraction
- **[Visualization & Mapping.md](Visualization_&_Mapping.md)** - Viewer application and map management
- **[Troubleshooting.md](Troubleshooting.md)** - Common issues and solutions

## Getting Started Steps

### 1. Clone the Repository
```bash
git clone git@github.com:cbenjamin23/moos-ivp-uav-base.git
cd moos-ivp-uav-base
git submodule update --init --recursive
```

### 2. Choose Your Simulation Approach

This project supports two simulation approaches:

- **MOOS-IvP Simulator (Lightweight)**: Uses built-in MOOS-IvP vehicle simulator with pMarineViewer
  - Recommended for beginners and basic autonomy testing
  - Faster setup, no ArduPilot or Gazebo required
  - Good for mission planning and behavior development
  
- **ArduPilot SITL + Gazebo (Full Simulation)**: Uses ArduPilot's Software-In-The-Loop with Gazebo
  - Full flight dynamics and 3D visualization
  - Closer to real hardware behavior
  - Requires additional installation (ArduPilot, Gazebo, plugins)

See [Installation & Setup.md](Installation_&_Setup.md) for details on both approaches.

### 3. Install Dependencies
Follow the installation instructions in [Installation & Setup.md](Installation_&_Setup.md) to install:
- MOOS-IVP core (required)
- MOOS IVP SWARM Toolbox (if needed)
- **Optional (for ArduPilot SITL + Gazebo approach only):**
  - ArduPilot
  - Gazebo Ionic simulator
- MAVSDK (included as submodule, required for ArduPilot integration)

### 4. Configure Environment
Set up bash aliases and environment variables as described in the main documentation.

### 5. Run Your First Mission
Refer to [System Launch Guide.md](System_Launch_Guide.md) for commands to launch vehicles and ground stations.

## Need Help?

If you encounter issues, check [Troubleshooting.md](Troubleshooting.md) for common problems and solutions.

For questions about specific components, refer to the relevant documentation files listed above.
