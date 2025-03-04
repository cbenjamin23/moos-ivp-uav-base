#ifndef FIRE_MISSION_SCORER_H
#define FIRE_MISSION_SCORER_H

#include <string>
#include <vector>
#include <map>
#include <set>
#include "MOOS/libMOOS/MOOSLib.h"
#include "FireSet.h"

class FireMissionScorer
{
public:
    // Constructor
    FireMissionScorer();

    // Initialization with mission parameters
    void Initialize(int totalFires, double deadlineSeconds, double totalCoverageArea);

    // Calculate score directly from FireSet
    double CalculateScoreFromFireSet(const FireSet &fireSet, double coveragePercentage);

    // Original methods for backward compatibility
    void SetCoveragePercentage(double percentage);

    // Get detailed score components
    double GetCompletenessScore() const { return m_completenessScore; }
    double GetTimeEfficiencyScore() const { return m_timeEfficiencyScore; }
    double GetCoverageScore() const { return m_coverageScore; }
    double GetRedundantDetectionPenalty() const { return m_redundantDetectionPenalty; }

    // Publish score to MOOSDB
    bool PublishScore(std::function <void(std::string, std::string)> reportFnc);

    // Save score to file
    bool SaveScoreToFile(const std::string &filename);

    // Get a formatted summary
    std::string GetScoreSummary();

    bool isScoreCalculated() const { return m_scoreCalculated; }

private:
    // Calculate scores internally (shared by both calculation methods)
    void CalculateScoreComponents(int detectedCount, int totalDetections, double latestDetectionTime);

    // Mission parameters
    int m_totalFires;
    double m_deadline;
    double m_totalArea;
    double m_coveragePercentage;

    // Detected fires
    unsigned int m_totalFiresDetected;
    unsigned int m_totalFiresDetections;
    double m_latestDetectionTime;
    double m_avgDetectionTime;
    double m_medianDetectionTime;

    // Score components
    double m_completenessScore;
    double m_timeEfficiencyScore;
    double m_coverageScore;
    double m_redundantDetectionPenalty;
    double m_totalScore;

    // Flag to track if score has been calculated
    bool m_scoreCalculated;
};

#endif
