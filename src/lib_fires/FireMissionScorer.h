#ifndef FIRE_MISSION_SCORER_H
#define FIRE_MISSION_SCORER_H

#include <string>
#include <vector>
#include <map>
#include <set>
#include "MOOS/libMOOS/MOOSLib.h"
#include "FireSet.h"
#include <functional>


class FireMissionScorer
{
public:
    FireMissionScorer();

    // init with mission parameters
    void init(int totalFires, double deadlineSeconds, double totalCoverageArea);

    double calculateScoreFromFireSet(const FireSet &fireSet, bool imputeTime = false);

    void setCoveragePercentage(double pct) { m_coveragePercentage = pct; };
    double getCoveragePercentage() const { return m_coveragePercentage; }

    // Get detailed score components
    double GetCompletenessScore() const { return m_completenessScore; }
    double GetTimeEfficiencyScore() const { return m_timeEfficiencyScore; }
    double GetCoverageScore() const { return m_coverageScore; }
    double GetRedundantDetectionPenalty() const { return m_redundantDetectionPenalty; }

    // save and publish score to MOOSDB
    bool SaveScoreToFile(const std::string &filename);
    bool PublishScore(std::function<void(std::string, std::string)> reportFnc);

    // Get a formatted summary
    std::string GetScoreSummary();
    bool isScoreCalculated() const { return m_scoreCalculated; }

private:
    // Calculate scores internally (shared by both calculation methods)
    void calculateScoreComponents(int detectedCount, int totalDetections, 
                                 double averageDetectionTime,
                                 double medianDetectionTime,
                                 double latestDetectionTime,
                                 bool imputeTime=false );

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
