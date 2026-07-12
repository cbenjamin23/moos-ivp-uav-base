# pArduBridge Documentation

- [Reference](README.md) — canonical configuration, MOOS interface, commands, policies, telemetry, and lifecycle semantics.
- [Quick start](QUICKSTART.md) — build, connect, verify telemetry, and issue basic commands.
- [Command reference](../UAV_Mission_ARDU_Commands.md) — operator-facing command distinctions and examples.
- [Architecture](ARCHITECTURE.md) — threading, control ownership, state, and confirmation design.
- [Hardware qualification](HARDWARE_QUALIFICATION.md) — indoor Pi/flight-controller bench, outdoor props-off qualification, and controlled-flight boundary.

Source entry points:

- `src/pArduBridge/ArduBridge.cpp` — MOOS boundary and bridge state.
- `src/pArduBridge/UAV_Model.cpp` — MAVSDK, policy, telemetry, and commands.
- `src/pArduBridge/ModeConfirmationTracker.h` — telemetry confirmation timeout.
- `src/pArduBridge/ArduBridge_Info.cpp` — built-in `--help`, `--example`, and `--interface` text.

The current app directory intentionally contains production code only. Plane and Copter completed a one-time logged SITL regression; hardware qualification remains required before flight.
