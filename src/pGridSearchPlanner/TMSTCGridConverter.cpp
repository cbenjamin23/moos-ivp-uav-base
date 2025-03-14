#include "TMSTCGridConverter.h"
#include <iostream>
#include <fstream>
#include <cmath>

#include "Logger.h"

TMSTCGridConverter::TMSTCGridConverter()
    : m_sensorRadius(0.0), m_regionWidth(0), m_regionHeight(0),
      m_spanningWidth(0), m_spanningHeight(0), m_gridsConverted(false)
{
}

TMSTCGridConverter::TMSTCGridConverter(const XYPolygon &searchRegion, double sensorRadius,
                                       const std::vector<XYPolygon> &ignoredRegions,
                                       const std::vector<XYPoint> &vehiclePositions)
    : m_searchRegion(searchRegion), m_sensorRadius(sensorRadius),
      m_ignoredRegions(ignoredRegions), m_vehiclePositions(vehiclePositions),
      m_gridsConverted(false)
{
    convert2MatGrids();
    m_gridsConverted = true;
}

void TMSTCGridConverter::setSearchRegion(const XYPolygon &searchRegion)
{
    m_searchRegion = searchRegion;
    m_gridsConverted = false;
}

void TMSTCGridConverter::setSensorRadius(double sensorRadius)
{
    m_sensorRadius = sensorRadius;
    m_gridsConverted = false;
}

void TMSTCGridConverter::setIgnoreRegions(const std::vector<XYPolygon> &ignoredRegions)
{
    m_ignoredRegions = ignoredRegions;
    m_gridsConverted = false;
}

void TMSTCGridConverter::setVehiclePositions(const std::vector<XYPoint> &vehiclePositions)
{
    m_vehiclePositions = vehiclePositions;
}

void TMSTCGridConverter::transformGrid()
{
    if(m_searchRegion.size() == 0)
    {
        Logger::warning("Search region polygon is empty!");
        return;
    }
    convert2MatGrids();
}

Mat TMSTCGridConverter::getRegionGrid() const
{
    return m_regionGrid;
}

Mat TMSTCGridConverter::getSpanningGrid() const
{
    return m_spanningGrid;
}

std::vector<XYPoint> TMSTCGridConverter::getRegionGridCenters() const
{
    return m_regionGridCenters;
}

std::vector<XYPoint> TMSTCGridConverter::getSpanningGridCenters() const
{
    return m_spanningGridCenters;
}

bool TMSTCGridConverter::isGridsConverted() const
{
    return m_gridsConverted;
}

XYSegList TMSTCGridConverter::pathToSegList(const std::vector<std::pair<int, int>> &path) const
{
    XYSegList segList;
    for (const auto &cell : path)
    {
        int row = cell.second;
        int col = cell.first;
        if (row >= 0 && row < m_regionHeight && col >= 0 && col < m_regionWidth)
        {
            // Calculate center of the cell in region grid (using 2 * sensorRadius)
            double x = m_boundingBox.get_min_x() + (col + 0.5) * 2 * m_sensorRadius;
            double y = m_boundingBox.get_min_y() + (row + 0.5) * 2 * m_sensorRadius;
            segList.add_vertex(x, y);
        }
    }
    return segList;
}
XYPoint TMSTCGridConverter::getVehicleRegionPosition(const XYPoint &pos) const{
    
    XYPoint null;
    null.clear(); // to make invalid point
    
    if (!pos.valid())
        return null; // Skip invalid points and return a default XYPoint

    int col = static_cast<int>((pos.get_vx() - m_boundingBox.get_min_x()) / (2 * m_sensorRadius));
    int row = static_cast<int>((pos.get_vy() - m_boundingBox.get_min_y()) / (2 * m_sensorRadius));
    if (row >= 0 && row < m_regionHeight && col >= 0 && col < m_regionWidth)
        return {static_cast<double>(col), static_cast<double>(row)};

    return null;
}
XYPoint TMSTCGridConverter::getVehicleSpanningPosition(const XYPoint &pos) const{
    XYPoint null;
    null.clear(); // to make invalid point

    if (!pos.valid())
        return null; // Skip invalid points and return a default XYPoint

    int col = static_cast<int>((pos.get_vx() - m_boundingBox.get_min_x()) / (4 * m_sensorRadius));
    int row = static_cast<int>((pos.get_vy() - m_boundingBox.get_min_y()) / (4 * m_sensorRadius));
    if (row >= 0 && row < m_spanningHeight && col >= 0 && col < m_spanningWidth)
        return {static_cast<double>(col), static_cast<double>(row)};
    
    return null;
}   

std::vector<std::pair<int, int>> TMSTCGridConverter::getVehicleRegionPositions() const
{
    std::vector<std::pair<int, int>> positions;
    for (const auto &pos : m_vehiclePositions)
    {
        XYPoint coord = getVehicleRegionPosition(pos);
        if(!coord.valid())
            continue;

        positions.emplace_back(coord.get_vx(), coord.get_vy());
        
    }
    return positions;
}

std::vector<std::pair<int, int>> TMSTCGridConverter::getVehicleSpanningPositions() const
{
    std::vector<std::pair<int, int>> positions;
    for (const auto &pos : m_vehiclePositions)
    {
        XYPoint coord = getVehicleSpanningPosition(pos);
        if(!coord.valid())
            continue;

        positions.emplace_back(coord.get_vx(), coord.get_vy());
    }
    return positions;
}

bool TMSTCGridConverter::saveRegionGridToFile(const std::string &filename) const
{
    std::ofstream outfile(filename);
    if (!outfile.is_open())
    {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return false;
    }

    // Write dimensions
    outfile << m_regionHeight << " " << m_regionWidth << "\n";

    // Write grid data
    for (int row = 0; row < m_regionHeight; ++row)
    {
        for (int col = 0; col < m_regionWidth; ++col)
        {
            outfile << m_regionGrid[row][col];
        }
        outfile << "\n";
    }

    outfile.close();
    return true;
}

bool TMSTCGridConverter::saveSpanningGridToFile(const std::string &filename) const
{
    std::ofstream outfile(filename);
    if (!outfile.is_open())
    {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return false;
    }

    // Write dimensions
    outfile << m_spanningHeight << " " << m_spanningWidth << "\n";

    // Write grid data
    for (int row = 0; row < m_spanningHeight; ++row)
    {
        for (int col = 0; col < m_spanningWidth; ++col)
        {
            outfile << m_spanningGrid[row][col];
        }
        outfile << "\n";
    }

    outfile.close();
    return true;
}

void TMSTCGridConverter::convert2MatGrids()
{
    if(m_gridsConverted)
        return;

    // Step 1: Get the bounding box of the search region
    m_boundingBox = getBoundingBox();

    // Step 2: Calculate grid dimensions using 2 * sensorRadius (ensure even numbers)
    double boxWidth = m_boundingBox.getLengthX();
    double boxHeight = m_boundingBox.getLengthY();
    m_regionWidth = static_cast<int>(std::ceil(boxWidth / (2 * m_sensorRadius)));
    m_regionHeight = static_cast<int>(std::ceil(boxHeight / (2 * m_sensorRadius)));
    // Ensure dimensions are even
    if (m_regionWidth % 2 != 0)
        m_regionWidth++;
    if (m_regionHeight % 2 != 0)
        m_regionHeight++;

    // Step 3: Initialize region grid and centers
    m_regionGrid = Mat(m_regionHeight, std::vector<int>(m_regionWidth, 0));
    m_regionGridCenters.clear();
    populateRegionGrid();

    // Step 4: Create downsampled spanning grid and centers
    m_spanningWidth = m_regionWidth / 2;
    m_spanningHeight = m_regionHeight / 2;
    m_spanningGrid = Mat(m_spanningHeight, std::vector<int>(m_spanningWidth, 0));
    m_spanningGridCenters.clear();
    createSpanningGrid();

    m_gridsConverted = true;
}

XYSquare TMSTCGridConverter::getBoundingBox() const
{
    double minX = m_searchRegion.get_min_x();
    double maxX = m_searchRegion.get_max_x();
    double minY = m_searchRegion.get_min_y();
    double maxY = m_searchRegion.get_max_y();
    return XYSquare(minX, maxX, minY, maxY);
}

void TMSTCGridConverter::populateRegionGrid()
{
    for (int row = 0; row < m_regionHeight; ++row)
    {
        for (int col = 0; col < m_regionWidth; ++col)
        {
            // Calculate center of the cell using 2 * sensorRadius
            double x = m_boundingBox.get_min_x() + (col + 0.5) * 2 * m_sensorRadius;
            double y = m_boundingBox.get_min_y() + (row + 0.5) * 2 * m_sensorRadius;

            // 1 - free , 0 - occupied cell
            double z = 0;

            // Check if the cell center is inside the search region
            if (m_searchRegion.contains(x, y))
            {
                // Check if the cell center is inside any ignored region
                bool isIgnored = false;
                for (const auto &ignored : m_ignoredRegions)
                {
                    if (ignored.contains(x, y))
                    {
                        isIgnored = true;
                        break;
                    }
                }
                // Set as free (1) if not ignored, otherwise occupied (0)
                m_regionGrid[row][col] = isIgnored ? 0 : 1;
                z = isIgnored ? 0 : 1;
            }

            m_regionGridCenters.emplace_back(x, y, z);

            // Cells outside the search region are already initialized as occupied (0)
        }
    }
}

void TMSTCGridConverter::createSpanningGrid()
{
    for (int row = 0; row < m_spanningHeight; ++row)
    {
        for (int col = 0; col < m_spanningWidth; ++col)
        {
            // Calculate center of the downsampled spanning cell (covers 2x2 region grid cells, so 4 * sensorRadius)
            double x = m_boundingBox.get_min_x() + (col + 0.5) * 4 * m_sensorRadius;
            double y = m_boundingBox.get_min_y() + (row + 0.5) * 4 * m_sensorRadius;

            // 1 - free , 0 - occupied cell
            double z = 0;

            // Count free cells in the 2x2 region
            int freeCount = 0;
            for (int i = 0; i < 2; ++i)
            {
                for (int j = 0; j < 2; ++j)
                {
                    int regionRow = row * 2 + i;
                    int regionCol = col * 2 + j;
                    if (m_regionGrid[regionRow][regionCol] == 1)
                    {
                        freeCount++;
                    }
                }
            }
            // Cell is free (1) if at least half (2 or more) of the 2x2 cells are free
            m_spanningGrid[row][col] = (freeCount >= 2) ? 1 : 0;
            z = (freeCount >= 2) ? 1 : 0;

            m_spanningGridCenters.emplace_back(x, y, z);
        }
    }
}
