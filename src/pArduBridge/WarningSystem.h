#pragma once

#include <iostream>
#include <functional>
#include <unordered_map>
#include <string>
#include <chrono>
#include <shared_mutex>
#include <mutex>
#include <unordered_set>


#include "../lib_common/Logger.h"


class WarningSystem
{
public:
    using CallbackType = std::function<void(const std::string &)>;

    WarningSystem(CallbackType MOOSReportCallback = nullptr, CallbackType MOOSRetractCallback = nullptr)
        : MOOSReportCallback(MOOSReportCallback), MOOSRetractCallback(MOOSRetractCallback) {}

    void queue_monitorWarningForXseconds(const std::string &warningKey, double seconds)
    {
        Logger::warning("WarningSystem: " + warningKey);
        auto start_time = std::chrono::steady_clock::now();
        auto expiration_time = start_time + std::chrono::milliseconds(static_cast<int>(seconds * 1000));

        std::function<bool()> condition = [expiration_time]()
        {
            return std::chrono::steady_clock::now() < expiration_time;
        };

        {
            std::unique_lock lock(mutex_);
            timeBasedWarnings.insert(warningKey); // time-based
        }

        queue_monitorCondition(warningKey, condition);
    }

    // Registers a condition to be monitored
    void queue_monitorCondition(const std::string &warningKey,
                                std::function<bool()> condition,
                                CallbackType reportCallback = nullptr,
                                CallbackType retractCallback = nullptr)
    {

        if (reportCallback == nullptr)
        {
            reportCallback = MOOSReportCallback;
        }
        if (retractCallback == nullptr)
        {
            retractCallback = MOOSRetractCallback;
        }

        std::unique_lock lock(mutex_);
        monitoredConditions[warningKey] = {condition, reportCallback, retractCallback};
        warningsActive[warningKey] = false; // Initialize as inactive
    }

    // Periodically called
    void checkConditions()
    {
        std::unique_lock lock(mutex_);
        auto it = monitoredConditions.begin();
        while (it != monitoredConditions.end())
        {
            const auto warningKey = it->first;
            auto &conditionData = it->second;
            auto &condition = conditionData.condition;
            auto &reportCallback = conditionData.reportCallback;
            auto &retractCallback = conditionData.retractCallback;

            if (condition())
            {
                if (!warningsActive[warningKey])
                {
                    if (reportCallback)
                        reportCallback(warningKey);    //  report callback
                    warningsActive[warningKey] = true; //  active
                }
                ++it;
            }
            else
            {
                if (warningsActive[warningKey])
                {
                    if (retractCallback)
                        retractCallback(warningKey);    // retract callback
                    warningsActive[warningKey] = false; // inactive
                }
                // Remove only time-based warnings
                if (timeBasedWarnings.count(warningKey))
                {
                    it = monitoredConditions.erase(it);
                    timeBasedWarnings.erase(warningKey);
                }
                else
                {
                    ++it;
                }
            }
        }
    }

private:
    struct ConditionData
    {
        std::function<bool()> condition;
        CallbackType reportCallback;
        CallbackType retractCallback;
    };

    CallbackType MOOSReportCallback;
    CallbackType MOOSRetractCallback;

    std::unordered_map<std::string, ConditionData> monitoredConditions;
    std::unordered_map<std::string, bool> warningsActive;

    std::unordered_set<std::string> timeBasedWarnings; // Track time-based warnings
    mutable std::shared_mutex mutex_;                  // Protects shared data
};
