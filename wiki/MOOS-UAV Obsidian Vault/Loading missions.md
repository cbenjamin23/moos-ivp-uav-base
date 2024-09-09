

load a generic waypoint mission from 

		~/ardupilot/Tools/autotest/Generic_Missions/


The PID values from Mission planner is as follows in the image
![[Pasted image 20240829151142.png]] where the green indicate the important change from the default values.

Also change under  Servo Yaw
		
		Dampening: 0.1 
		Integral 0.05


## Changing the simulation speed[¶](https://ardupilot.org/dev/docs/sitl-with-gazebo.html#changing-the-simulation-speed "Link to this heading")

By default Gazebo will attempt to run the simulation with a Real Time Factor (RTF) of 1. To increase the simulation speed add the following XML block into the world file just after the opening <world> element:

> <physics name="1ms" type="ignore">
>   <max_step_size>0.001</max_step_size>
>   <real_time_factor>-1.0</real_time_factor>
> </physics>

Then set the simulation speed-up parameter in MAVProxy

> MANUAL> param set SIM_SPEEDUP 1

