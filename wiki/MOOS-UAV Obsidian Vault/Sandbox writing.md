

I want to create a bash file that starts a simulator for me.


#### Run Gazebo
It should open one terminal (terminator) and run the following command:

		gz sim -v4 -r skywalker_x8_runway.sdf


#### Run ArduPilot SITL
Then is should open another terminal for the following command

	sim_vehicle.py -v ArduPlane --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/skywalker_x8.param --console --map


#### Run Missionplanner
Finally it should the following commands but ensure that it runs in the background:

		cd ~/Mission_Planner 
		mono MissionPlanner.exe 





In all these terminal (which should be terminator), if the program crashes or halts, the window should not quit.