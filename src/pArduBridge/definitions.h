#pragma once


#include <iostream>

// Warnings

constexpr char WARNING_NO_UAV_CONNECTION[] = "No connection to UAV. Please connect to UAV first.";
constexpr char WARNING_UAV_NOT_ARMED[] = "UAV is not armed. Please arm UAV first.";


constexpr char WARNING_TIMED_OUT[] = "Timed out waiting for response from UAV.";


// Constants

constexpr double WARNING_DURATION = 10.0;  // seconds

constexpr int IN_AIR_HIGHT_THRESHOLD = 5; // meters
constexpr int DISTANCE_TO_HEADING_WAYPOINT = 5000; // meters


constexpr double MAX_CENTRIPITAL_ACC_TURN = 10; // m/s^2

constexpr int AUX_SWITCH_LOW = 0;
constexpr int AUX_SWITCH_HIGH = 2;
constexpr int MAV_CMD_DO_AUX_FUNCTION_ARDUPILOT = 218;
constexpr int AUX_FUNC_PRECISION_LOITER = 39;
