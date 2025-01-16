#pragma once


// ArduPilot Spesific Commands
enum MAV_CMD_ARDUPILOT{

    MAV_CMD_GUIDED_CHANGE_SPEED     = 43000, // Change flight speed at a given rate. This slews the vehicle at a controllable rate between it's previous speed and the new one
    MAV_CMD_GUIDED_CHANGE_ALTITUDE  = 43001, // Change target altitude at a given rate. This slews the vehicle at a controllable rate between it's previous altitude and the new one. 
    MAV_CMD_GUIDED_CHANGE_HEADING   = 43002, // Change to target heading at a given rate, overriding previous heading/s. This slews the vehicle at a controllable rate between it's previous heading and the new one. Exiting GUIDED returns aircraft to normal behaviour defined elsewhere
};


enum HEADING_TYPE{
    HEADING_TYPE_COURSE_OVER_GROUND = 0, // Course over ground
    HEADING_TYPE_HEADING            = 1, // Heading in Body frame
    HEADING_TYPE_DEFAULT            = 2, // Default
};





