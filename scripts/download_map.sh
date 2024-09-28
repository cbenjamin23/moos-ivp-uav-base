#!/bin/bash

# Default values
datum_lat=63.3975168
datum_lon=10.1435321

latStart=63.42997
lonStart=9.98427
latEnd=63.34077
lonEnd=10.21430
zoom=16
tileServerIDGMAPS=0
tileServerIDGSAT=1
stitchFormat=".png"
baseDir="~/moos-ivp-uav/Maps"  # Base directory for saving maps
folderName="NTNU_UAV_Airport"  # Folder name under baseDir
tilesDir="~/moos-ivp-uav/Maps/anaximap"  # Directory for downloaded tiles

# Function to display help message
display_help() {
    echo "Usage: $0 [options]"
    echo
    echo "Options:"
    echo "  --datum_lat LAT           Set the datum latitude (default: $datum_lat)"
    echo "  --datum_lon LON           Set the datum longitude (default: $datum_lon)"
    echo "  --latStart LAT            Set the starting latitude (default: $latStart)"
    echo "  --lonStart LON            Set the starting longitude (default: $lonStart)"
    echo "  --latEnd LAT              Set the ending latitude (default: $latEnd)"
    echo "  --lonEnd LON              Set the ending longitude (default: $lonEnd)"
    echo "  --zoom ZOOM               Set the zoom level (default: $zoom)"
    echo "  --baseDir DIR             Set the base directory (default: $baseDir)"
    echo "  --folderName NAME         Set the output folder name (default: $folderName)"
    echo "  --tilesDir DIR            Set the directory for downloading tiles (default: $tilesDir)"
    echo "  -h, --help                Show this help message"
    echo
    echo "Example usage:"
    echo "  $0 --datum_lat 63.5 --datum_lon 10.2 --latStart 63.52 --lonStart 9.77 --baseDir ~/maps --folderName Custom_UAV_Airport"
    exit 0
}

# Check for optional arguments and override defaults if present
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --datum_lat) datum_lat="$2"; shift ;;
        --datum_lon) datum_lon="$2"; shift ;;
        --latStart) latStart="$2"; shift ;;
        --lonStart) lonStart="$2"; shift ;;
        --latEnd) latEnd="$2"; shift ;;
        --lonEnd) lonEnd="$2"; shift ;;
        --zoom) zoom="$2"; shift ;;
        --baseDir) baseDir="$2"; shift ;;
        --folderName) folderName="$2"; shift ;;
        --tilesDir) tilesDir="$2"; shift ;;
        -h|--help) display_help ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Expand ~ to the home directory
baseDir=$(eval echo "$baseDir")
tilesDir=$(eval echo "$tilesDir")

# Full output path combining base directory and folder name
outputFolder="$baseDir/$folderName"

# Print internal variables for confirmation
echo "Using the following settings:"
echo "datum_lat: $datum_lat"
echo "datum_lon: $datum_lon"
echo "latStart: $latStart"
echo "lonStart: $lonStart"
echo "latEnd: $latEnd"
echo "lonEnd: $lonEnd"
echo "zoom: $zoom"
echo "baseDir: $baseDir"
echo "folderName: $folderName"
echo "tilesDir: $tilesDir"
echo "Full output folder: $outputFolder"

# Ask for user confirmation
read -p "Are these settings correct? (y/n): " confirmation
if [[ "$confirmation" != "y" ]]; then
    echo "Exiting script. No changes made."
    exit 1
fi

# Save current directory
cwd=$(pwd)

# Move to AnaxiMap directory
cd ~/moos-ivp-uav/Maps/anaximap

# Create the output folder if it doesn't exist
mkdir -p "$outputFolder"

# Function to process maps and generate info and tif files
process_map() {
    local tileServerID=$1
    local outputPrefix=$2
    local savePath="$outputFolder/$outputPrefix"

    echo "Processing map with tile server ID: $tileServerID"

    # Move to tilesDir for downloading and processing the tiles
    cd "$tilesDir"

    # Run the map generation script with the correct working directory
    python3 ~/AnaxiMap/tsdl.py "$latStart" "$lonStart" "$latEnd" "$lonEnd" "$zoom" "$tileServerID" --stitchFormat="$stitchFormat"

    # Locate the generated .info file in the tiles/raw directory
    infoFile=$(find "$tilesDir" -name "*.info" | head -n 1)

    if [[ ! -f "$infoFile" ]]; then
        echo "Error: .info file not found!"
        return 1
    fi

    # Extract lat/lon values from the .info file
    lat_north=$(grep "lat_north" "$infoFile" | cut -d '=' -f 2 | xargs)
    lat_south=$(grep "lat_south" "$infoFile" | cut -d '=' -f 2 | xargs)
    lon_east=$(grep "lon_east" "$infoFile" | cut -d '=' -f 2 | xargs)
    lon_west=$(grep "lon_west" "$infoFile" | cut -d '=' -f 2 | xargs)
    mapFile=$(grep "mapFile" "$infoFile" | cut -d '=' -f 2 | xargs)

    # Ensure that mapFile ends with the stitchFormat, regardless of the current extension
    mapFile="${mapFile%.*}${stitchFormat}"
    
    # Create the new .info file
    infoOutput="$savePath.info"
    echo "lat_north = $lat_north" > "$infoOutput"
    echo "lat_south = $lat_south" >> "$infoOutput"
    echo "lon_east  = $lon_east" >> "$infoOutput"
    echo "lon_west  = $lon_west" >> "$infoOutput"
    echo "datum_lat = $datum_lat" >> "$infoOutput"
    echo "datum_lon = $datum_lon" >> "$infoOutput"
    
    # Convert the map file to a .tif file and move to the output folder
    mapPath="$tilesDir/tiles/$mapFile"
    tifOutput="$savePath.tif"

    echo "Converting $mapPath to $tifOutput..."
    convert "$mapPath" "$tifOutput"

    # Return to the tilesDir in case further processing is needed
    cd "$tilesDir"
}

# Process GMAPS (NTNU_UAV_Airport_gmaps)
process_map "$tileServerIDGMAPS" "NTNU_UAV_Airport_gmaps"

# Process GSAT (NTNU_UAV_Airport_gsat)
process_map "$tileServerIDGSAT" "NTNU_UAV_Airport_gsat"

# Return to the original directory
cd "$cwd"

echo "Map processing completed."
