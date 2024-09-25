#pragma once

#include <iostream>
#include <functional>
#include <unordered_map>
#include <string>

#include <chrono>
#include <mutex>

class WarningSystem {
public:
    // Function signatures for warning reporting and retraction
    using CallbackType = std::function<void(const std::string&)>;


    WarningSystem(CallbackType MOOSReportCallback = nullptr, CallbackType MOOSRetractCallback = nullptr) 
        : MOOSReportCallback(MOOSReportCallback), MOOSRetractCallback(MOOSRetractCallback) {}

    void monitorWarningForXseconds(const std::string& warningKey, double seconds) {

        
         // Get the current time as the start time
        auto start_time = std::chrono::steady_clock::now();

        // Create a condition that checks whether the current time is within the duration
        std::function<bool()> condition = [start_time, seconds]() {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
            return elapsed_seconds < seconds;  // True if still within the time limit
        };

        monitorCondition(warningKey, condition);
    }

    // Register a condition to be monitored
    void monitorCondition(const std::string& warningKey, 
                          std::function<bool()> condition, 
                          CallbackType reportCallback = nullptr, 
                          CallbackType retractCallback = nullptr) {
        

        if (reportCallback == nullptr) {
            reportCallback = MOOSReportCallback;
        }
        if (retractCallback == nullptr) {
            retractCallback = MOOSRetractCallback;
        }
        monitoredConditions[warningKey] = {condition, reportCallback, retractCallback};
    }

    // This method is called periodically (e.g., in a loop or timer)
    void checkConditions() {
        for (auto& [warningKey, conditionData] : monitoredConditions) {
            auto& condition = conditionData.condition;
            auto& reportCallback = conditionData.reportCallback;
            auto& retractCallback = conditionData.retractCallback;

            if (condition() && !warningsActive[warningKey]) {
                reportCallback(warningKey);
                warningsActive[warningKey] = true;
            } else if (!condition() && warningsActive[warningKey]) {
                retractCallback(warningKey);
                warningsActive[warningKey] = false;
            }
        }
    }

private:
    
    CallbackType MOOSReportCallback;
    CallbackType MOOSRetractCallback;


    struct ConditionData {
        std::function<bool()> condition;
        CallbackType reportCallback;
        CallbackType retractCallback;
    };

    std::unordered_map<std::string, ConditionData> monitoredConditions;
    std::unordered_map<std::string, bool> warningsActive;
};

