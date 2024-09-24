

# ArduPilot

The spesific parameter list used for ArduPilot Plane can be found in [here](https://ardupilot.org/plane/docs/parameters.html)  (web) of [here](https://autotest.ardupilot.org/Parameters/ArduPlane/apm.pdef.xml) (xml).
and look for plane version **v4.1.2**

For All parameter across all vehicles look [here](https://ardupilot.org/dev/docs/mavlink-get-set-params.html) which also informs about the Mavlink Interface with ArduPilot


It will not compile if you are using python 3.10 or newer. To fix the error generetad do the following:

#### Modify the UAVCAN Library Code

Since the error is due to an outdated reference in the `uavcan` library, you can manually update the Python code in the `uavcan` library to reflect the correct import.

##### Open the file causing the error 

```bash
	code ~/ardupilot/modules/uavcan/libuavcan/dsdl_compiler/pyuavcan/uavcan/transport.py
```
	

 ##### Find the line:
    
    
```python
class ArrayValue(BaseValue, collections.MutableSequence):
```
   
##### Change the import to use `collections.abc`:
    
    
```python
from collections.abc import MutableSequence
```
    
    
##### Modify the class definition as follows:
    
    
```python
class ArrayValue(BaseValue, MutableSequence):
```


Ny run simulator again



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

