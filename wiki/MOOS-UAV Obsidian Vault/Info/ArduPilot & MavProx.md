

# ArduPilot

The spesific parameter list used for ArduPilot Plane can be found in [here](https://ardupilot.org/plane/docs/parameters.html)  (web) of [here](https://autotest.ardupilot.org/Parameters/ArduPlane/apm.pdef.xml) (xml).

For All parameter across all vehicles look [here](https://ardupilot.org/dev/docs/mavlink-get-set-params.html) which also informs about the Mavlink Interface with ArduPilot


# MavProxy

- UDP forwarding:
	After it has started use the command
	
		output add <destination_IP_address>:<Port>
	to connect to the IP above. Eg `120.0.0.1:14550` for localhost



## Changing the simulation speed[¶](https://ardupilot.org/dev/docs/sitl-with-gazebo.html#changing-the-simulation-speed "Link to this heading")

By default Gazebo will attempt to run the simulation with a Real Time Factor (RTF) of 1. To increase the simulation speed add the following XML block into the world file just after the opening <world> element in the file of the X8 model:

> <physics name="1ms" type="ignore">
>   <max_step_size>0.001</max_step_size>
>   <real_time_factor>-1.0</real_time_factor>
> </physics>

Then set the simulation speed-up parameter in MAVProxy

> MANUAL> param set SIM_SPEEDUP 1

