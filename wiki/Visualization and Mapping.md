You can download new background using the script `download_map.sh`

For that you need the tool AnaxiMap, that can be cloned from [here](https://github.com/HeroCC/AnaxiMap) in home folder

The tool to find the lat-long coordinates for the map can be found [here](https://tools.geofabrik.de/calc/#type=geofabrik_standard&bbox=9.952507,63.339781,10.324227,63.446049)

The tool will download 2 version of the map, one satellite image, and one terrain.



# pArduBridge

The parameters 
```
WP_LOITER_RAD    100
WP_RADIUS        50
```
defined in the parameter list (*sim.param*) for ardupilot has `meters` as units.
In the behavior file these differ as distance is measured in local XY coordinates.

**Observation:** A distance of 1 meter in real life is a distance of 2 in local XY coordinates in moos.  


