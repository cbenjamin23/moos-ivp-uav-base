---
source paper: file:///home/steve/Downloads/zolich2015unmanned%20-%20UAS%20description.pdf
---
### Sensors

Where does all the important sensor info come from 
	 Does the payload computer have to poll the sensors, or is it sent directly from the autopilot (Ardupilot)

	- GPS (Position), Speed, Altitude
- Answer:
	- Uses a MAVlink telemetry protocol
	- 3 input output telemetry ports (2x UART, 1x USB)
