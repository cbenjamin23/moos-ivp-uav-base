#ifndef TMSTC_GRID_CONVERTER_H
#define TMSTC_GRID_CONVERTER_H

#include <vector>
#include <string>
#include <cmath>
#include "XYPolygon.h"
#include "XYSquare.h"
#include "XYSegList.h"
#include "XYPoint.h"

typedef std::vector<std::vector<int>> Mat;

class TMSTCGridConverter
{
public:
    // Default constructor
    TMSTCGridConverter();

    // Constructor with initial parameters
    TMSTCGridConverter(const XYPolygon &searchRegion, double sensorRadius,
                       const std::vector<XYPolygon> &ignoredRegions,
                       const std::vector<XYPoint> &vehiclePositions);

    // Setter methods
    void setSearchRegion(const XYPolygon &searchRegion);
    void setSensorRadius(double sensorRadius);
    void setIgnoreRegions(const std::vector<XYPolygon> &ignoredRegions);
    void setVehiclePositions(const std::vector<XYPoint> &vehiclePositions);
    void transformGrid();

    // Getter methods
    Mat getRegionGrid() const;
    Mat getSpanningGrid() const;
    std::vector<XYPoint> getRegionGridCenters() const;
    std::vector<XYPoint> getSpanningGridCenters() const;
    XYSegList pathToSegList(const std::vector<std::pair<int, int>> &path) const;

    XYPoint getVehicleRegionPosition(const XYPoint &pos) const;
    std::vector<std::pair<int, int>> getVehicleRegionPositions() const;
    XYPoint getVehicleSpanningPosition(const XYPoint &pos) const;
    std::vector<std::pair<int, int>> getUniqueVehicleSpanningCoordinates() const;
    bool saveRegionGridToFile(const std::string &filename) const;
    bool saveSpanningGridToFile(const std::string &filename) const;
    bool isGridsConverted() const;

    XYSegList regionCoords2XYSeglisMoos(std::vector<std::pair<int, int>> regionCoords) const;
    XYSegList spanningCoords2XYSeglisMoos(std::vector<std::pair<int, int>> spanningCoords) const;

    XYPoint regionCoord2XYPointMoos(int col, int row) const;
    XYPoint spanningCoord2XYPointMoos(int col, int row) const;

private:
    XYPolygon m_searchRegion;                   // The search area polygon
    double m_sensorRadius;                      // Sensor coverage radius
    std::vector<XYPolygon> m_ignoredRegions;    // Ignored regions within the search area
    std::vector<XYPoint> m_vehiclePositions;    // Vehicle positions as XYPoint
    XYSquare m_boundingBox;                     // Bounding box of the search region
    Mat m_regionGrid;                           // Full resolution grid (1 = free, 0 = occupied)
    Mat m_spanningGrid;                         // Downsampled spanning grid (1/4 area)
    std::vector<XYPoint> m_regionGridCenters;   // Centers of region grid cells
    std::vector<XYPoint> m_spanningGridCenters; // Centers of downsampled spanning grid cells
    int m_regionWidth, m_regionHeight;          // Dimensions of region grid
    int m_spanningWidth, m_spanningHeight;      // Dimensions of downsampled spannnign grid
    bool m_gridsConverted;                      // Flag to check if grids have been converted

    // Private helper methods
    void convert2MatGrids();
    XYSquare getBoundingBox() const;
    void populateRegionGrid();
    void createSpanningGrid();
};

#endif // TMSTC_GRID_CONVERTER_H
