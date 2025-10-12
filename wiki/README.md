# Getting Started with MOOS-IVP UAV

This documentation provides guidance on setting up and running the MOOS-IVP UAV system.

## Quick Navigation

### Essential Setup Documentation
- **[MAIN System architecture.md](MAIN%20System%20architecture.md)** - Core system installation and setup instructions
  - Clone repository and submodules
  - Install MOOS-IVP
  - Install MOOS IVP SWARM Toolbox
  - Setup ArduPilot, Gazebo, and related tools

### Key Components
- **[Install, Build & Run MAVSDK.md](Install,%20Build%20&%20Run%20MAVSDK.md)** - MAVSDK installation and usage
- **[ArduPilot & MavProx.md](ArduPilot%20&%20MavProx.md)** - ArduPilot configuration, parameters, Mission Planner setup, and swarm capabilities
- **[Launching System.md](Launching%20System.md)** - How to launch missions and ground stations
- **[ardupilot_gazebo plugin.md](ardupilot_gazebo%20plugin.md)** - Gazebo plugin details and version information

### Hardware and Configuration
- **[Skywalker X8.md](Skywalker%20X8.md)** - Skywalker X8 specific information

### Additional Resources
- **[Data Management.md](Data%20Management.md)** - Managing mission data
- **[pMarineViewer - MOOS  IVP.md](pMarineViewer%20-%20MOOS%20%20IVP.md)** - Viewer application
- **[Troubleshooting.md](Troubleshooting.md)** - Common issues and solutions

## Getting Started Steps

### 1. Clone the Repository
```bash
git clone git@github.com:cbenjamin23/moos-ivp-uav-base.git
cd moos-ivp-uav-base
git submodule update --init --recursive
```

### 2. Install Dependencies
Follow the installation instructions in [MAIN System architecture.md](MAIN%20System%20architecture.md) to install:
- MOOS-IVP core
- MOOS IVP SWARM Toolbox (if needed)
- ArduPilot
- Gazebo Ionic simulator
- MAVSDK (included as submodule)

### 3. Configure Environment
Set up bash aliases and environment variables as described in the main documentation.

### 4. Run Your First Mission
Refer to [Launching System.md](Launching%20System.md) for commands to launch vehicles and ground stations.

## Need Help?

If you encounter issues, check [Troubleshooting.md](Troubleshooting.md) for common problems and solutions.

For questions about specific components, refer to the relevant documentation files listed above.
