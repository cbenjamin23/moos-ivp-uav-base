#!/bin/bash

### function to extract telemetry data from shoreside alog files
extract_GCS_telemetry() {
    if ! command -v aloggrep &> /dev/null; then
        echo "aloggrep could not be found."
        return 1
    fi

    local alog_file

    # if argument is provided, use it as the alog file
    if [ $# -gt 0 ]; then
        alog_file=("$1")
    else
        alog_file=(XLOG_*.alog)
    fi

    if [ ! -f "$alog_file" ]; then
        echo "File $alog_file not found."
        return 1
    fi

    local VARS="NODE_REPORT* SURVEY_UPDATE*"
    local output_file="GCS_telemetry_data.csv"

    echo "Processing $alog_file ..."
    echo "time;variable;value" > $output_file
    aloggrep "$alog_file" $VARS --tvv --css --sort >> $output_file

    echo "Data extraction complete. Output written to $output_file"
}

# extract telemetry data from the alog files of vehicles

extract_drone_telemetry() {
    # Split the .alog files into individual folders
    alogsplit *.alog

    numeric_output_file="telemetry_data_numeric.csv"
    categorical_output_file="telemetry_data_categorical.csv"

    # Change directory to the folder ending with *alvtmp
    cd *alvtmp

    # Define the numeric and categorical variables HERE!!!!
    VARS_numeric="NAV_* DESIRED_*"
    VARS_categorical="AUTOPILOT_MODE TOWAYPT_UPDATE SURVEY_UPDATE*"

    # Create the output files for telemetry data
    numeric_output_file="../$numeric_output_file"
    categorical_output_file="../$categorical_output_file"

    # Initialize files with headers
    echo "time,variable,value" > $numeric_output_file
    echo "time,variable,value" > $categorical_output_file

    # Process numeric and categorical variables
    for var in $VARS_numeric $VARS_categorical; do
        # Append ".klog" if the variable doesn't already end with .klog
        if [[ "$var" != *.klog ]]; then
            var="${var}.klog"
        fi
        nav_file=$var
        
        # Check if the .klog file exists
        if [[ -f "$nav_file" ]]; then        
            echo "Processing $nav_file..."
            
            # Determine whether it's a numeric or categorical variable
            base_name=$(basename "$nav_file" .klog)
            
            if [[ "$VARS_categorical" == *"$base_name"* ]]; then
                # Categorical variable: append to the categorical telemetry file
                aloggrep "$nav_file" "$base_name" --tvv --css --sort >> $categorical_output_file
            else
                # Numeric variable: append to the numeric telemetry file
                aloggrep "$nav_file" "$base_name" --tvv --css --sort >> $numeric_output_file
            fi
        else
            echo "No matching files for $nav_file"
        fi
    done

    cd -

    echo "Data extraction complete. Output written to $numeric_output_file and $categorical_output_file"
}