# Getting Started with MOOS-IVP UAV

This documentation provides guidance on setting up and running the MOOS-IVP UAV system.

## Quick Navigation

### Essential Setup Documentation
- **[Installation and Setup.md](Installation%20and%20Setup.md)** - Core system installation and setup instructions
  - Clone repository and submodules
  - Install MOOS-IVP
  - Install MOOS IVP SWARM Toolbox
  - Setup ArduPilot, Gazebo, and related tools

### Key Components
- **[MAVSDK Setup and Usage.md](MAVSDK%20Setup%20and%20Usage.md)** - MAVSDK installation and usage
- **[ArduPilot & MavProx.md](ArduPilot%20&%20MavProx.md)** - ArduPilot configuration, parameters, Mission Planner setup, and swarm capabilities
- **[System Launch Guide.md](System%20Launch%20Guide.md)** - How to launch missions and ground stations
- **[Gazebo Plugin Configuration.md](Gazebo%20Plugin%20Configuration.md)** - Gazebo plugin details and version information

### Hardware and Configuration
- **[Skywalker X8.md](Skywalker%20X8.md)** - Skywalker X8 specific information

### Additional Resources
- **[Telemetry Data Extraction.md](Telemetry%20Data%20Extraction.md)** - Managing mission data and telemetry extraction
- **[Visualization and Mapping.md](Visualization%20and%20Mapping.md)** - Viewer application and map management
- **[Troubleshooting.md](Troubleshooting.md)** - Common issues and solutions

## Getting Started Steps

### 1. Clone the Repository
```bash
git clone git@github.com:cbenjamin23/moos-ivp-uav-base.git
cd moos-ivp-uav-base
git submodule update --init --recursive
```

### 2. Install Dependencies
Follow the installation instructions in [Installation and Setup.md](Installation%20and%20Setup.md) to install:
- MOOS-IVP core
- MOOS IVP SWARM Toolbox (if needed)
- ArduPilot
- Gazebo Ionic simulator
- MAVSDK (included as submodule)

### 3. Configure Environment
Set up bash aliases and environment variables as described in the main documentation.

### 4. Run Your First Mission
Refer to [System Launch Guide.md](System%20Launch%20Guide.md) for commands to launch vehicles and ground stations.

## Need Help?

If you encounter issues, check [Troubleshooting.md](Troubleshooting.md) for common problems and solutions.

For questions about specific components, refer to the relevant documentation files listed above.
