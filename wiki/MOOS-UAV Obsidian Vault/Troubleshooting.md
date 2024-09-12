


## SITL fails

Run the command 

		~/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh
This wil setup things again


## Gazebo not running/ dying after start

There might be a gazebo server/client running. Identify the PID with the command

		ps aux | grep gz
and then kill it using `kill <PID>`

For MissionPlanner: 
		
		ps aux | grep mono
and then kill it using `kill <PID>`



## How to search in files:

	Example from dune
	
		grep -ri “matrix” <path_to_dune_src>
		- ./Maneuver/CoverArea/Task.cpp: Math::Matrix m_rows; // etc

![[Pasted image 20240903114151.png]]