#!/bin/bash


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
            aloggrep "$nav_file" "$base_name" --tvv --css >> $categorical_output_file
        else
            # Numeric variable: append to the numeric telemetry file
            aloggrep "$nav_file" "$base_name" --tvv --css  >> $numeric_output_file
        fi
    else
        echo "No matching files for $nav_file"
    fi
done

cd -

echo "Data extraction complete. Output written to $numeric_output_file and $categorical_output_file"