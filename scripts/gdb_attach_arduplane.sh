#!/bin/bash

# Find the correct PID of the running arduplane process (excluding xterm and grep)
pid=$(ps aux | grep "$HOME/ardupilot/build/sitl/bin/[a]rduplane" | grep -v 'xterm' | grep -v 'grep' | awk '{print $2}' | head -n 1)

# Check if PID was found
if [ -z "$pid" ]; then
  echo "No running arduplane process found."
  exit 1
fi

# Print the PID that was found
echo "Attaching gdb to arduplane process with PID: $pid"

# Run gdb and attach to the process
sudo gdb -p "$pid"
