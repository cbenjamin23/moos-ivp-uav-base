#pragma once


#include <iostream>

// Warnings

constexpr char WARNING_NO_UAV_CONNECTION[] = "No connection to UAV. Please connect to UAV first.";
constexpr char WARNING_UAV_NOT_ARMED[] = "UAV is not armed. Please arm UAV first.";


constexpr char WARNING_TIMED_OUT[] = "Timed out waiting for response from UAV.";


constexpr double WARNING_DURATION = 10.0;  // seconds