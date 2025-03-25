#!/bin/bash
# get_region_xy.sh

# Function to convert lat/lon points to local XY offsets.
# The output is a colon-separated list of points in "X,Y" format.


# Set locale to C to ensure decimal point is a period.
export LC_NUMERIC=C

function get_region_xy() {
    config_file="$1"
    
    # Read the flag from the config
    use_moos_region=$(yq eval '.missionParams.use_moos_region' "$config_file")
    
    if [ "$use_moos_region" = "true" ]; then
        # If using MOOS region, simply return the region_XY value.
        region_xy=$(yq eval '.missionParams.region_XY' "$config_file")
        echo "$region_xy"
    else
        # Otherwise, convert the lat/lon list to local XY offsets.
        datum_lat=$(yq eval '.moos.datum.lat' "$config_file")
        datum_lon=$(yq eval '.moos.datum.lon' "$config_file")
        echo "Datum: lat=$datum_lat, lon=$datum_lon" >&2
        
        # Earth's radius (m)
        R=6378137
        # Conversion factor for degrees to radians
        rad_factor=$(echo "3.141592653589793/180" | bc -l)
        # cosine of datum latitude in radians (awk for trigonometry)
        cos_datum=$(awk -v lat="$datum_lat" 'BEGIN { print cos(lat * 3.141592653589793/180) }')
        
        # Read the region_latlon block as a multi-line scalar.
        region_string=$(yq eval '.missionParams.region_lonlat' "$config_file")
        
        result=""
        
        # Process each line.
        while IFS= read -r line; do
            # Skip empty lines.
            if [ -z "$line" ]; then
                continue
            fi
            # Remove leading/trailing whitespace.
            line=$(echo "$line" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')
            
            # format is lon, lat.
            lon=$(echo "$line" | cut -d',' -f1 | tr -d ' ')
            lat=$(echo "$line" | cut -d',' -f2 | tr -d ' ')
            
            # differences in degrees
            delta_lat=$(echo "$lat - $datum_lat" | bc -l)
            delta_lon=$(echo "$lon - $datum_lon" | bc -l)
            
            # Convert differences to offsets in meters.
            # offset_y: north-south direction
            offset_y=$(echo "$delta_lat * $rad_factor * $R" | bc -l)
            # offset_x: east-west direction (scaled by cosine of latitude)
            offset_x=$(echo "$delta_lon * $rad_factor * $R * $cos_datum" | bc -l)
            
            # Format offsets to 1 decimal (rounded)
            offset_x_int=$(printf "%.1f" "$offset_x")
            offset_y_int=$(printf "%.1f" "$offset_y")
            
            # Format as "X,Y"
            pair="${offset_x_int},${offset_y_int}"
            if [ -z "$result" ]; then
                result="$pair"
            else
                result="$result:$pair"
            fi
        done <<< "$region_string" 
        
        # Return the colon-separated list of offsets.
        echo "$result"
    fi
}

# Example usage:
# ./get_region_xy.sh path/to/config.yaml


function get_global_val() {
    
    config_file=$1
    field_name=$2
    val=$(yq eval ".$field_name" "$config_file")
    if [ "$val" != "null" ]; then
        echo "$val"
        return 0
    fi
    echo "Error: Field '$field_name' not found in config file: $config_file." >&2
    return 1
}

function get_global_val_in_moosDistance() {
    config_file=$1
    field_name=$2
    val=$(get_global_val $config_file $field_name)
    if [ $? -ne 0 ]; then return 1; fi
    echo $(echo "$val * 2" | bc)   
}


function get_val_by_drone_name() {
    config_file=$1
    drone_name=$2
    field_name=$3

    total_drones=$(yq eval '.drones | length' "$config_file")

    for i in $(seq 0 $(($total_drones - 1))); do
         current_name=$(yq eval ".drones[$i].name" "$config_file")

        if [ "$current_name" = "$drone_name" ]; then

             val=$(yq eval ".drones[$i].$field_name" "$config_file")
            if [ "$val" != "null" ]; then
                echo "$val"
                return 0
            fi
            echo "Error: Field '$field_name' not found for drone with name '$drone_name'." >&2
            return 1
        fi
    done

    # print an error and return 1
    echo "Error: Drone with name '$drone_name' not found." >&2
    return 1
}



# Function to get IP address of the current device
get_ipaddr() {

    
    # Try common methods to get the current device's IP
    local ip_addr
    
    # Method 1: using ip command (modern Linux)
    ip_addr=$(ip -4 addr show scope global | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | tail -n 1)
    
    # Method 2: fallback using ifconfig (older systems)
    if [[ -z "$ip_addr" ]]; then
        ip_addr=$(ifconfig | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v '127.0.0.1' | head -n 1)
    fi
    
    # Method 3: fallback using hostname (some systems)
    if [[ -z "$ip_addr" ]]; then
        ip_addr=$(hostname -I | awk '{print $1}')
    fi
    
    if [[ -n "$ip_addr" ]]; then
        echo "$ip_addr"
        return 0
    else
        echo "Error: Could not determine IP address" >&2
        return 1
    fi
    
    
    # If key doesn't match or no value found
    echo "Error: Key '$key' not handled or value not found" >&2 
    return 1
}