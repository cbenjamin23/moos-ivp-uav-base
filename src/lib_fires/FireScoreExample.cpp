#include "FireMissionScorer.h"
#include "../lib_fires/FireSet.h"
#include <iostream>

// Example of how to use the FireMissionScorer
int main(int argc, char *argv[])
{
    // Example 1: Using FireSet for score calculation
    std::cout << "\n=== EXAMPLE 1: Using FireSet for scoring ===\n";

    // Create a FireSet with two fires
    FireSet fireSetA;
    std::string warning;

    // Add fires to the FireSet
    fireSetA.addFire("f01", "undiscovered", 100, 200, 0, warning);
    fireSetA.addFire("f02", "undiscovered", 300, 400, 0, warning);

    // Get fires and simulate discoveries
    Fire fire1 = fireSetA.getFire("f01");
    fire1.setState(Fire::DISCOVERED);
    fire1.setTimeDiscovered(300.0); // 5 minutes
    fire1.setDiscoverer("uav1");
    fire1.incDiscoverCnt();
    fireSetA.modFire(fire1);

    Fire fire2 = fireSetA.getFire("f02");
    fire2.setState(Fire::DISCOVERED);
    fire2.setTimeDiscovered(300.0); // 5 minutes
    fire2.setDiscoverer("uav2");
    fire2.incDiscoverCnt();
    fireSetA.modFire(fire2);

    // Calculate score using FireSet
    FireMissionScorer scorerA;
    scorerA.Initialize(2, 600.0, 1000.0);                               // 2 fires, 10-minute deadline, 1000 m² area
    double scoreA = scorerA.calculateScoreFromFireSet(fireSetA, 100.0); // 100% coverage

    std::cout << "Score from FireSet: " << scoreA << std::endl;
    std::cout << scorerA.GetScoreSummary() << std::endl;

    // Example 2: Multiple detections with FireSet
    std::cout << "\n=== EXAMPLE 3: Multiple detections with FireSet ===\n";

    FireSet fireSetD;
    fireSetD.addFire("f01", "undiscovered", 100, 200, 0, warning);
    fireSetD.addFire("f02", "undiscovered", 300, 400, 0, warning);

    // First fire detected twice
    Fire fire1D = fireSetD.getFire("f01");
    fire1D.setState(Fire::DISCOVERED);
    fire1D.setTimeDiscovered(300.0); // 5 minutes
    fire1D.setDiscoverer("uav1");
    fire1D.incDiscoverCnt(); // First detection
    fire1D.incDiscoverCnt(); // Second detection (redundant)
    fireSetD.modFire(fire1D);

    // Second fire detected twice
    Fire fire2D = fireSetD.getFire("f02");
    fire2D.setState(Fire::DISCOVERED);
    fire2D.setTimeDiscovered(290.0); // 4.83 minutes
    fire2D.setDiscoverer("uav1");
    fire2D.incDiscoverCnt(); // First detection
    fire2D.incDiscoverCnt(); // Second detection (redundant)
    fireSetD.modFire(fire2D);

    // Calculate score
    FireMissionScorer scorerD;
    scorerD.Initialize(2, 600.0, 1000.0);
    double scoreD = scorerD.calculateScoreFromFireSet(fireSetD, 100.0);

    std::cout << "Score with redundant detections: " << scoreD << std::endl;
    std::cout << scorerD.GetScoreSummary() << std::endl;

    return 0;
}
