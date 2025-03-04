#include "FireMissionScorer.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>

FireMissionScorer::FireMissionScorer()
{
    m_totalFires = 0;
    m_deadline = 0.0;
    m_totalArea = 0.0;
    m_coveragePercentage = 0.0;

    m_totalFiresDetected = 0;
    m_totalFiresDetections = 0;
    m_latestDetectionTime = 0.0;
    m_avgDetectionTime = 0.0;
    m_medianDetectionTime = 0.0;
    

    m_completenessScore = 0.0;
    m_timeEfficiencyScore = 0.0;
    m_coverageScore = 0.0;
    m_redundantDetectionPenalty = 0.0;
    m_totalScore = 0.0;

    m_scoreCalculated = false;
}

void FireMissionScorer::Initialize(int totalFires, double deadlineSeconds, double totalCoverageArea)
{
    m_totalFires = totalFires;
    m_deadline = deadlineSeconds;
    m_totalArea = totalCoverageArea;
    m_scoreCalculated = false;
}

double FireMissionScorer::CalculateScoreFromFireSet(const FireSet &fireSet, double coveragePercentage)
{
    m_coveragePercentage = std::min(100.0, std::max(0.0, coveragePercentage));

    // Get statistics from FireSet
    m_totalFiresDetected = fireSet.getTotalFiresDiscovered();

    
    m_totalFiresDetections = 0;
    std::vector<double> detectionTimes;
    // Get all detected fires and count total detections
    std::vector<Fire> fires = fireSet.getFires();
    for (const Fire &fire : fires)
    {
        if (fire.getState() == Fire::DISCOVERED)
        {
            m_totalFiresDetections += fire.getDiscoverCnt();
            detectionTimes.push_back(fire.getTimeDiscovered());
        }
    }

    // use algorithm to find latest detection time
    if (!detectionTimes.empty()){
        std::sort(detectionTimes.begin(), detectionTimes.end());
        m_latestDetectionTime = detectionTimes.back();

        // calculate average detection time
        double sum = std::accumulate(detectionTimes.begin(), detectionTimes.end(), 0.0);
        m_avgDetectionTime = sum / detectionTimes.size();

        // calculate median detection time
        if (detectionTimes.size() % 2 == 0)
            m_medianDetectionTime = (detectionTimes[detectionTimes.size() / 2 - 1] + detectionTimes[detectionTimes.size() / 2]) / 2;
        else
            m_medianDetectionTime = detectionTimes[detectionTimes.size() / 2];
    }

    double detectonTime = m_latestDetectionTime;

    // Calculate score components
    CalculateScoreComponents(m_totalFiresDetected, m_totalFiresDetections, detectonTime);

    return m_totalScore;
}



void FireMissionScorer::CalculateScoreComponents(int detectedCount, int totalDetections, double latestDetectionTime)
{
    // Calculate completeness score (50 points max)
    m_completenessScore = (static_cast<double>(detectedCount) / m_totalFires) * 50.0;

    // Calculate time efficiency score (50 points max)
    m_timeEfficiencyScore = 0.0;
    if (detectedCount == m_totalFires)
    {
        // Calculate time efficiency
        double timeRatio = latestDetectionTime / m_deadline;
        m_timeEfficiencyScore = (1.0 - std::min(1.0, timeRatio)) * 50.0;
    }

    // Calculate coverage score (10 points max)
    m_coverageScore = (m_coveragePercentage / 100.0) * 10.0;

    // Calculate redundant detections penalty
    int redundantDetections = totalDetections - detectedCount;

    if (detectedCount > 0)
    {
        double avgRedundancyPerFire = static_cast<double>(redundantDetections) / detectedCount;
        m_redundantDetectionPenalty = avgRedundancyPerFire * 2.0;
    }
    else
        m_redundantDetectionPenalty = 0.0;

    // Calculate total score
    m_totalScore = m_completenessScore + m_timeEfficiencyScore + m_coverageScore - m_redundantDetectionPenalty;

    // Ensure score is between 0 and 100
    m_totalScore = std::min(100.0, std::max(0.0, m_totalScore));

    m_scoreCalculated = true;
}



bool FireMissionScorer::PublishScore(std::function <void(std::string, std::string)> reportFnc)
{
    if (!m_scoreCalculated)
        return false;
    
    if (reportFnc)
    {
        reportFnc("FIRE_MISSION_TOTAL_SCORE", doubleToString(m_totalScore));
        reportFnc("FIRE_MISSION_COMPLETENESS_SCORE", doubleToString(m_completenessScore));
        reportFnc("FIRE_MISSION_TIME_EFFICIENCY_SCORE", doubleToString(m_timeEfficiencyScore));
        reportFnc("FIRE_MISSION_COVERAGE_SCORE", doubleToString(m_coverageScore));
        reportFnc("FIRE_MISSION_REDUNDANT_PENALTY", doubleToString(m_redundantDetectionPenalty));
        reportFnc("FIRE_MISSION_FIRES_DETECTED", doubleToString(m_totalFiresDetected));
        reportFnc("FIRE_MISSION_DETECTION_COUNT", doubleToString(m_totalFiresDetections));
        reportFnc("FIRE_MISSION_LATEST_DETECTION_TIME", doubleToString(m_latestDetectionTime));
        reportFnc("FIRE_MISSION_AVERAGE_DETECTION_TIME", doubleToString(m_avgDetectionTime));
        reportFnc("FIRE_MISSION_MEDIAN_DETECTION_TIME", doubleToString(m_medianDetectionTime));
        reportFnc("FIRE_MISSION_SUMMARY", GetScoreSummary());
        return true;
    }

    return false;
}

bool FireMissionScorer::SaveScoreToFile(const std::string &filename)
{
    if (!m_scoreCalculated)
        return false;
    

    std::ofstream file(filename);
    if (!file.is_open())
        return false;
    
    file << GetScoreSummary();
    file.close();
    return true;
}

std::string FireMissionScorer::GetScoreSummary()
{
    if (!m_scoreCalculated)
        return "Score not calculated yet.";


    std::stringstream ss;
    ss << "======= FIRE MISSION SCORE SUMMARY =======\n";
    ss << "Total Score: " << std::fixed << std::setprecision(2) << m_totalScore << " / 100\n";
    ss << "----------------------------------------\n";
    ss << "Completeness Score: " << std::fixed << std::setprecision(2) << m_completenessScore << " / 50\n";
    ss << "Time Efficiency Score: " << std::fixed << std::setprecision(2) << m_timeEfficiencyScore << " / 50\n";
    ss << "Coverage Score: " << std::fixed << std::setprecision(2) << m_coverageScore << " / 10\n";
    ss << "Redundant Detection Penalty: -" << std::fixed << std::setprecision(2) << m_redundantDetectionPenalty << "\n";
    ss << "----------------------------------------\n";
    ss << "Fires Detected: " << m_totalFiresDetected << " / " << m_totalFires << "\n";
    ss << "Latest Detection Time: " << std::fixed << std::setprecision(2) << m_latestDetectionTime << " seconds\n";
    ss << "Average Detection Time: " << std::fixed << std::setprecision(2) << m_avgDetectionTime << " seconds\n";
    ss << "Median Detection Time: " << std::fixed << std::setprecision(2) << m_medianDetectionTime << " seconds\n";
    ss << "----------------------------------------\n";
    ss << "Total Detections: " << m_totalFiresDetections << "\n";
    ss << "Area Coverage: " << std::fixed << std::setprecision(2) << m_coveragePercentage << "%\n";
    ss << "========================================\n";

    return ss.str();
}
