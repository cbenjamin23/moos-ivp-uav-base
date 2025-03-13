#include <vector>
#include <string>
#include <cmath>
#include "XYPolygon.h"
#include "XYSquare.h"
#include "XYSegList.h"
#include "XYPoint.h"

typedef std::vector<std::vector<int>> Mat;

class TMSTCGridConverter {
public:
    // Default constructor
    TMSTCGridConverter() : m_sensorRadius(0.0), m_regionWidth(0), m_regionHeight(0), 
                           m_downsampledWidth(0), m_downsampledHeight(0) {}

    // Constructor with initial parameters
    TMSTCGridConverter(const XYPolygon& searchRegion, double sensorRadius, 
                       const std::vector<XYPolygon>& ignoredRegions, 
                       const std::vector<XYPoint>& vehiclePositions)
        : m_searchRegion(searchRegion), m_sensorRadius(sensorRadius), 
          m_ignoredRegions(ignoredRegions), m_vehiclePositions(vehiclePositions) {
        initializeGrids();
    }

    // Setter methods
    void setSearchRegion(const XYPolygon& searchRegion) {
        m_searchRegion = searchRegion;
    }

    void setSensorRadius(double sensorRadius) { 
        m_sensorRadius = sensorRadius; 
    }

    void addIgnoreRegion(const XYPolygon& ignoredRegion) {
        m_ignoredRegions.push_back(ignoredRegion);
    }

    void setVehiclePositions(const std::vector<XYPoint>& vehiclePositions) {
        m_vehiclePositions = vehiclePositions;
    }

    void convertGrid() { 
        initializeGrids(); 
    }

    // Get the region grid (full resolution)
    Mat getRegionGrid() const { return m_regionGrid; }

    // Get the downsampled grid (1/4 area)
    Mat getDownsampledGrid() const { return m_downsampledGrid; }

    // Get region grid cell centers as XYPoints (z = unique ID)
    std::vector<XYPoint> getRegionGridCenters() const { return m_regionGridCenters; }

    // Get downsampled grid cell centers as XYPoints (z = unique ID)
    std::vector<XYPoint> getDownsampledGridCenters() const { return m_downsampledGridCenters; }

    // Convert a path (vector of indices in region grid) to XYSegList of cell centers
    XYSegList pathToSegList(const std::vector<std::pair<int, int>>& path) const {
        XYSegList segList;
        for (const auto& cell : path) {
            int row = cell.second;
            int col = cell.first;
            if (row >= 0 && row < m_regionHeight && col >= 0 && col < m_regionWidth) {
                // Calculate center of the cell in region grid (using 2 * sensorRadius)
                double x = m_boundingBox.get_min_x() + (col + 0.5) * 2 * m_sensorRadius;
                double y = m_boundingBox.get_min_y() + (row + 0.5) * 2 * m_sensorRadius;
                segList.add_vertex(x, y);
            }
        }
        return segList;
    }

    // Get vehicle positions in region grid coordinates
    std::vector<std::pair<int, int>> getVehicleRegionPositions() const {
        std::vector<std::pair<int, int>> positions;
        for (const auto& pos : m_vehiclePositions) {
            if (!pos.valid()) continue; // Skip invalid points
            int col = static_cast<int>((pos.get_vx() - m_boundingBox.get_min_x()) / (2 * m_sensorRadius));
            int row = static_cast<int>((pos.get_vy() - m_boundingBox.get_min_y()) / (2 * m_sensorRadius));
            if (row >= 0 && row < m_regionHeight && col >= 0 && col < m_regionWidth) {
                positions.emplace_back(col, row);
            }
        }
        return positions;
    }

    // Get vehicle positions in downsampled grid coordinates
    std::vector<std::pair<int, int>> getVehicleDownsampledPositions() const {
        std::vector<std::pair<int, int>> positions;
        for (const auto& pos : m_vehiclePositions) {
            if (!pos.valid()) continue; // Skip invalid points
            int col = static_cast<int>((pos.get_vx() - m_boundingBox.get_min_x()) / (4 * m_sensorRadius));
            int row = static_cast<int>((pos.get_vy() - m_boundingBox.get_min_y()) / (4 * m_sensorRadius));
            if (row >= 0 && row < m_downsampledHeight && col >= 0 && col < m_downsampledWidth) {
                positions.emplace_back(col, row);
            }
        }
        return positions;
    }

    // Save the region grid to a file
    bool saveRegionGridToFile(const std::string& filename) const {
        std::ofstream outfile(filename);
        if (!outfile.is_open()) {
            std::cerr << "Error opening file for writing: " << filename << std::endl;
            return false;
        }

        // Write dimensions
        outfile << m_regionHeight << " " << m_regionWidth << "\n";

        // Write grid data
        for (int row = 0; row < m_regionHeight; ++row) {
            for (int col = 0; col < m_regionWidth; ++col) {
                outfile << m_regionGrid[row][col];
            }
            outfile << "\n";
        }

        outfile.close();
        return true;
    }

    // Save the downsampled grid to a file
    bool saveDownsampledGridToFile(const std::string& filename) const {
        std::ofstream outfile(filename);
        if (!outfile.is_open()) {
            std::cerr << "Error opening file for writing: " << filename << std::endl;
            return false;
        }

        // Write dimensions
        outfile << m_downsampledHeight << " " << m_downsampledWidth << "\n";

        // Write grid data
        for (int row = 0; row < m_downsampledHeight; ++row) {
            for (int col = 0; col < m_downsampledWidth; ++col) {
                outfile << m_downsampledGrid[row][col];
            }
            outfile << "\n";
        }

        outfile.close();
        return true;
    }
private:
    XYPolygon m_searchRegion;                    // The search area polygon
    double m_sensorRadius;                       // Sensor coverage radius
    std::vector<XYPolygon> m_ignoredRegions;     // Ignored regions within the search area
    std::vector<XYPoint> m_vehiclePositions;     // Vehicle positions as XYPoint
    XYSquare m_boundingBox;                      // Bounding box of the search region
    Mat m_regionGrid;                            // Full resolution grid (1 = free, 0 = occupied)
    Mat m_downsampledGrid;                       // Downsampled grid (1/4 area)
    std::vector<XYPoint> m_regionGridCenters;    // Centers of region grid cells
    std::vector<XYPoint> m_downsampledGridCenters; // Centers of downsampled grid cells
    int m_regionWidth, m_regionHeight;           // Dimensions of region grid
    int m_downsampledWidth, m_downsampledHeight; // Dimensions of downsampled grid

    // Initialize both the region grid and downsampled grid
    void initializeGrids() {
        // Step 1: Get the bounding box of the search region
        m_boundingBox = getBoundingBox();

        // Step 2: Calculate grid dimensions using 2 * sensorRadius (ensure even numbers)
        double boxWidth = m_boundingBox.getLengthX();
        double boxHeight = m_boundingBox.getLengthY();
        m_regionWidth = static_cast<int>(std::ceil(boxWidth / (2 * m_sensorRadius)));
        m_regionHeight = static_cast<int>(std::ceil(boxHeight / (2 * m_sensorRadius)));
        // Ensure dimensions are even
        if (m_regionWidth % 2 != 0) m_regionWidth++;
        if (m_regionHeight % 2 != 0) m_regionHeight++;

        // Step 3: Initialize region grid and centers
        m_regionGrid = Mat(m_regionHeight, std::vector<int>(m_regionWidth, 0));
        m_regionGridCenters.clear();
        populateRegionGrid();

        // Step 4: Create downsampled grid and centers
        m_downsampledWidth = m_regionWidth / 2;
        m_downsampledHeight = m_regionHeight / 2;
        m_downsampledGrid = Mat(m_downsampledHeight, std::vector<int>(m_downsampledWidth, 0));
        m_downsampledGridCenters.clear();
        createDownsampledGrid();
    }

    // Get the bounding box encapsulating the search region
    XYSquare getBoundingBox() const {
        double minX = m_searchRegion.get_min_x();
        double maxX = m_searchRegion.get_max_x();
        double minY = m_searchRegion.get_min_y();
        double maxY = m_searchRegion.get_max_y();
        return XYSquare(minX, maxX, minY, maxY);
    }

    // Populate the region grid with free (1) and occupied (0) states and store centers
    void populateRegionGrid() {
        for (int row = 0; row < m_regionHeight; ++row) {
            for (int col = 0; col < m_regionWidth; ++col) {
                // Calculate center of the cell using 2 * sensorRadius
                double x = m_boundingBox.get_min_x() + (col + 0.5) * 2 * m_sensorRadius;
                double y = m_boundingBox.get_min_y() + (row + 0.5) * 2 * m_sensorRadius;

                // 1 - free , 0 - occupied cell
                double z = 0;

                // Check if the cell center is inside the search region
                if (m_searchRegion.contains(x, y)) {
                    // Check if the cell center is inside any ignored region
                    bool isIgnored = false;
                    for (const auto& ignored : m_ignoredRegions) {
                        if (ignored.contains(x, y)) {
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

    // Create the downsampled grid by merging 2x2 cells from the region grid and store centers
    void createDownsampledGrid() {
        for (int row = 0; row < m_downsampledHeight; ++row) {
            for (int col = 0; col < m_downsampledWidth; ++col) {
                // Calculate center of the downsampled cell (covers 2x2 region grid cells, so 4 * sensorRadius)
                double x = m_boundingBox.get_min_x() + (col + 0.5) * 4 * m_sensorRadius;
                double y = m_boundingBox.get_min_y() + (row + 0.5) * 4 * m_sensorRadius;

                
                // 1 - free , 0 - occupied cell
                double z = 0;

                // Count free cells in the 2x2 region
                int freeCount = 0;
                for (int i = 0; i < 2; ++i) {
                    for (int j = 0; j < 2; ++j) {
                        int regionRow = row * 2 + i;
                        int regionCol = col * 2 + j;
                        if (m_regionGrid[regionRow][regionCol] == 1) {
                            freeCount++;
                        }
                    }
                }
                // Cell is free (1) if at least half (2 or more) of the 2x2 cells are free
                m_downsampledGrid[row][col] = (freeCount >= 2) ? 1 : 0;
                z = (freeCount >= 2) ? 1 : 0;
                
                m_downsampledGridCenters.emplace_back(x, y, z);
            }

        }
    }
};
