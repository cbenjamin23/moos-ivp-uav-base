#pragma once

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <optional> // C++17 for optional return type

class SetpointManager
{
private:
    using Clock = std::chrono::steady_clock;

    double desiredSpeed;
    double desiredCourse;
    double desiredAltitude;

    double prevSpeed;
    double prevCourse;
    double prevAltitude;

    std::optional<Clock::time_point> speedUpdatedAt;
    std::optional<Clock::time_point> courseUpdatedAt;
    std::optional<Clock::time_point> altitudeUpdatedAt;

    bool hasChanged(double prevValue, double newValue)
    {
        return prevValue != newValue;
    }

    double ageSeconds(const std::optional<Clock::time_point>& updatedAt,
                      Clock::time_point now) const
    {
        if (!updatedAt)
            return std::numeric_limits<double>::infinity();
        return std::chrono::duration<double>(now - *updatedAt).count();
    }

public:
    SetpointManager()
        : desiredSpeed(0.0), desiredCourse(0.0), desiredAltitude(0.0),
          prevSpeed(0.0), prevCourse(0.0), prevAltitude(0.0) {}

    // Method to update desired values
    void updateDesiredSpeed(double newSpeed)
    {
        desiredSpeed = newSpeed;
        speedUpdatedAt = Clock::now();
    }
    void updateDesiredCourse(double newHeading)
    {
        desiredCourse = newHeading;
        courseUpdatedAt = Clock::now();
    }
    void updateDesiredAltitude(double newAltitude)
    {
        desiredAltitude = newAltitude;
        altitudeUpdatedAt = Clock::now();
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

    bool allSetpointsReceived() const
    {
        return speedUpdatedAt.has_value() &&
               courseUpdatedAt.has_value() &&
               altitudeUpdatedAt.has_value();
    }

    double oldestSetpointAgeSeconds() const
    {
        const auto now = Clock::now();
        return std::max({
            ageSeconds(speedUpdatedAt, now),
            ageSeconds(courseUpdatedAt, now),
            ageSeconds(altitudeUpdatedAt, now)});
    }

    bool setpointsFresh(double timeoutSeconds) const
    {
        return allSetpointsReceived() &&
               oldestSetpointAgeSeconds() <= timeoutSeconds;
    }

    bool isValid() const
    {
        return !(desiredAltitude == 0.0 && desiredCourse == 0.0 && desiredSpeed == 0.0);
    }
};
