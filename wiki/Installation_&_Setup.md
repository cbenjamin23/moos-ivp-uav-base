# Installation & Setup

This repository contains a submodule for MAVSDK and helper scripts to set up a MOOS/IVP UAV development environment.

## Understanding Simulation Approaches

This project supports **two primary modes** controlled by the `useMoosSimPid` parameter in `missionConfig.yaml`:

### Mode 1: MOOS-IvP Simulator (`useMoosSimPid: true`)
   - ✅ Lightweight simulation using built-in MOOS-IvP vehicle simulator
   - ✅ Visualization with pMarineViewer
   - ✅ No pArduBridge, ArduPilot, or Gazebo required
   - ✅ Good for mission planning and MOOS-IvP behavior development
   - **Recommended for beginners**

### Mode 2: pArduBridge Integration (`useMoosSimPid: false`)
When using pArduBridge (`useMoosSimPid: false`), you can connect to:

**Option A: Physical Hardware**
   - Connect to a real drone with ArduPilot flight controller
   - Requires MAVSDK and pArduBridge
   - No simulation software needed

**Option B: ArduPilot SITL (Software Simulation)**
   - ✅ ArduPilot Software-In-The-Loop simulation
   - ✅ Requires ArduPilot installation and MAVSDK
   - ⚠️ **Gazebo is optional** - adds realistic physics and 3D visualization
   - Use Gazebo only if you need enhanced flight dynamics modeling

**For beginners:** Start with Mode 1 (MOOS-IvP Simulator). Only use Mode 2 with ArduPilot when testing with physical hardware or when you need ArduPilot-specific flight dynamics.

## Getting started

Clone the repository (using SSH) and initialize submodules:

```bash
git clone git@github.com:cbenjamin23/moos-ivp-uav-base.git
cd moos-ivp-uav-base
git submodule update --init --recursive
```

## Setup bash aliases

The file `scripts/setup_bash_aliases_moos.sh` defines useful aliases. Make it executable and source it from your shell startup file:

```bash
chmod +x ~/moos-ivp-uav-base/scripts/setup_bash_aliases_moos.sh
# then add to ~/.bashrc or source it directly
source ~/moos-ivp-uav-base/scripts/setup_bash_aliases_moos.sh
```

## Install MOOS‑IVP (GitHub)

MOOS/IVP was migrated from SVN. Clone the upstream repository and build:

```bash
git clone https://github.com/moos-ivp/moos-ivp.git
cd moos-ivp
git pull
./build.sh
```

Note: use the `-m` flag when building on a vehicle to only build non-GUI apps:

```bash
./build.sh -m
```

### Verify installation

Check that the main binaries are on your PATH:

```bash
which MOOSDB
which pHelmIvP
```

Example expected output:

    $ which MOOSDB
    /Users/you/moos-ivp/bin/MOOSDB
    $ which pHelmIvP
    /Users/you/moos-ivp/bin/pHelmIvP

You can also run an example mission:

```bash
cd ivp/missions/s1_alpha
pAntler --MOOSTimeWarp=10 alpha.moos
```

Add the moos-ivp-uav-base bin folder to your PATH in `~/.bashrc`:

```bash
PATH=$PATH:~/moos-ivp-uav-base/bin
export PATH
```

## MOOS/IVP Editor for VS Code

Install the MOOS/IVP editor extension for VS Code. One option is the provided VSIX file:

```bash
code --install-extension moos-ivp-editor-0.2.0.vsix
```

Guide: https://msis.github.io/2680notes/editors/vscode/10%20-%20Setting%20things%20up/#install-recommended-extensions

## Install MOOS/IVP SWARM Toolbox

This codebase is private (owned by pavlab-MIT) and requires access to clone. Contact `scnomeny@mit.edu` for access. If you do not hear back, try `mail@scnomeny.com`.

After cloning, build the library:

```bash
cd moos-ivp-swarm
./build.sh
```

Note: use `./build.sh -m` when building on a vehicle.

If build errors occur, fix them before retrying. See Troubleshooting.md for more info.

Update your environment variables (`PATH` and `IVP_BEHAVIOR_DIRS`) in `~/.bashrc`:

```bash
IVP_BEHAVIOR_DIRS=$IVP_BEHAVIOR_DIRS:~/moos-ivp-swarm/lib
export IVP_BEHAVIOR_DIRS

PATH=$PATH:~/moos-ivp-swarm/bin
export PATH
```

Verify the swarm tools are installed:

```bash
which pMediator
```

Expected output example:

    /Users/you/moos-ivp-swarm/bin/pMediator

---

## Optional: ArduPilot Installation

**⚠️ Only required if using pArduBridge (`useMoosSimPid: false`)**

This is needed when:
- Working with physical hardware (real drone)
- Using ArduPilot SITL for software simulation

Skip this section if you're using the MOOS-IvP Simulator (`useMoosSimPid: true`).

### ArduPilot (to run arduplane) with MavProxy

Location: `~/ardupilot/`

Install guide: https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux

- Run `git submodule update --init --recursive`
- Run `install-prereqs-ubuntu.sh` before switching branch
- CHECKOUT branch: `Plane-4.6.3`
- More info: ArduPilot & MavProx (link in repo)

### ardupilot_gazebo (Gazebo Sim / Ionic)

**Note:** Gazebo is **completely optional** even when using ArduPilot SITL. Install only if you want enhanced physics simulation and 3D visualization.

Install the Gazebo Sim (Ionic) simulator binary: https://gazebosim.org/docs/ionic/install_ubuntu/

Test it with:

```bash
gz sim -v4 -r shapes.sdf
```

Repository location: `~/gz_ws/ardupilot_gazebo/`

Clone:

```bash
git clone git@github.com:ArduPilot/ardupilot_gazebo.git
```

Add SITL models and Gazebo paths to your `~/.bashrc`:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH: \
  $HOME/SITL_Models/Gazebo/models: \
  $HOME/SITL_Models/Gazebo/worlds: \
  $HOME/moos-ivp-uav-base/GazeboSim/models: \
  $HOME/moos-ivp-uav-base/GazeboSim/worlds:
```

Important: build the Gazebo plugin with CMake in the plugin's build folder and run `make` before using it.

Gazebo version: this project uses Gazebo Sim 8 (Ionic). For more info on environment variables see:
https://answers.gazebosim.org//question/29153/some-questions-about-uri-in-sdf-files/

---

### Optional: Mission Planner / QGroundControl

**⚠️ Only required for physical hardware or advanced ArduPilot parameter tuning**

Two software options for interfacing with the flight controller parameters and tuning the plane physically. QGroundControl has Mac/Windows installation support and Mission Planner solely has Windows.

QGround_Control

Install guide: https://qgroundcontrol.com/

Mission_Planner

Install guide: https://ardupilot.org/planner/docs/mission-planner-installation.html

Configuration details in ArduPilot & MavProx.

---

## MAVSDK Installation

Location: `~/MAVSDK/`

**This step is required for both simulation approaches** when interfacing with ArduPilot (either in simulation or on hardware).

Follow MAVSDK setup instructions or use the `mavsdk_build_install` alias in `scripts/setup_bash_aliases_moos.sh`. See: **[MAVSDK Setup & Usage.md](MAVSDK_Setup_&_Usage.md)**

---

Relevant topics: Troubleshooting.md
