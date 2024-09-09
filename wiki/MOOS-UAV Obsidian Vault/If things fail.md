


## SITL fails

Run the command 

		~/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh
This wil setup things again


## Gazebo not running properly

There might be a gazebo server/client running. Identify the PID with the command

		ps aux | grep gz
and then kill it using `kill <PID>`

For MissionPlanner: 
		
		ps aux | grep mono
and then kill it using `kill <PID>`

