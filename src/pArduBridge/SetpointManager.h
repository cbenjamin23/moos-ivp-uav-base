#pragma once

#include <iostream>
#include <optional> // C++17 for optional return type

class SetpointManager
{
private:
    double desiredSpeed;
    double desiredCourse;
    double desiredAltitude;

    double prevSpeed;
    double prevCourse;
    double prevAltitude;

    bool hasChanged(double prevValue, double newValue)
    {
        return prevValue != newValue;
    }

public:
    SetpointManager()
        : desiredSpeed(0.0), desiredCourse(0.0), desiredAltitude(0.0),
          prevSpeed(0.0), prevCourse(0.0), prevAltitude(0.0) {}

    // Method to update desired values
    void updateDesiredSpeed(double newSpeed)
    {
        desiredSpeed = newSpeed;
    }
    void updateDesiredCourse(double newHeading)
    {
        desiredCourse = newHeading;
    }
    void updateDesiredAltitude(double newAltitude)
    {
        desiredAltitude = newAltitude;
    }

    // Polling function for desired values. Returns std::optional to indicate if value changed.
    std::optional<double> getDesiredSpeed()
    {
        if (hasChanged(prevSpeed, desiredSpeed))
        {
            prevSpeed = desiredSpeed;
            return desiredSpeed;
        }
        return std::nullopt; // No update
    }
    std::optional<double> getDesiredCourse()
    {
        if (hasChanged(prevCourse, desiredCourse))
        {
            prevCourse = desiredCourse;
            return desiredCourse;
        }
        return std::nullopt; // No update
    }
    std::optional<double> getDesiredAltitude()
    {
        if (hasChanged(prevAltitude, desiredAltitude))
        {
            prevAltitude = desiredAltitude;
            return desiredAltitude;
        }
        return std::nullopt; // No update
    }

    // Return all desired values unconditionally
    double readDesiredSpeed() const { return desiredSpeed; }
    double readDesiredCourse() const { return desiredCourse; }
    double readDesiredAltitudeAGL() const { return desiredAltitude; }

    bool isValid() const
    {
        return !(desiredAltitude == 0.0 && desiredCourse == 0.0 && desiredSpeed == 0.0);
    }
};
