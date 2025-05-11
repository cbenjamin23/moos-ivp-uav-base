import argparse
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

import re


FILTERTIME_OFFSETT = -150

DESIRED_PATH_VERTEX_SIZE = 15

# Function to parse the desired path
def parse_desired_path(desired_path):
    points = desired_path.split(':')
    x_coords, y_coords = [], []

    for point in points:
        x, y = point.split(',')
        x_coords.append(float(x))
        y_coords.append(float(y))

    return x_coords, y_coords

# Function to ensure directory exists
def ensure_directory_exists(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)

# Function to parse fires file and extract polygon and fire coordinates
def parse_fires_file(file_path):
    if not file_path or not os.path.exists(file_path):
        return None, None
    
    polygon_coords = None
    fires = []
    
    try:
        with open(file_path, 'r') as file:
            for line in file:
                line = line.strip()
                
                # Extract polygon coordinates
                if line.startswith('poly =') and 'pts=' in line:
                    match = re.search(r'pts=\{([^}]+)\}', line)
                    if match:
                        points_str = match.group(1)
                        polygon_coords = parse_desired_path(points_str)
                
                # Extract fire coordinates
                elif line.startswith('fire ='):
                    match = re.search(r'x=(-?\d+\.?\d*),\s*y=(-?\d+\.?\d*)', line)
                    if match:
                        x = float(match.group(1))
                        y = float(match.group(2))
                        
                        # Extract fire name if available
                        name_match = re.search(r'name=(\w+)', line)
                        name = name_match.group(1) if name_match else ""
                        
                        fires.append({'name': name, 'x': x, 'y': y})
    
    except Exception as e:
        print(f"Error parsing fires file: {e}")
        return None, None
    
    return polygon_coords, fires

# Function to automatically determine the target path from the data
def determine_target_path(categorical_data, helm_start, timeStart):
    if categorical_data is None:
        return None

    # print(f"Categorical data:\n {categorical_data}")
    # Filter the categorical data after helm_start or timeStart
    start_time = helm_start if helm_start is not None else timeStart
    filtered_categorical = categorical_data[categorical_data['time'] >= start_time]
    
    #sort filtered data after time
    filtered_categorical = filtered_categorical.sort_values(by='time')
    
    
    mode=None
    variable_towaypoint = None
    variable_survey = None
    
    # print(f"filtered categorical:\n {filtered_categorical}")
    # Iterate through the categorical data to find the last relevant mode and corresponding update
    
    desired_path = None
    for _, row in filtered_categorical.iterrows():

        if row['variable'] == 'AUTOPILOT_MODE':
            mode = row['value']
        elif row['variable'] == 'TOWAYPT_UPDATE':
            variable_towaypoint = row['value']
        elif row['variable'] == 'SURVEY_UPDATE':
            variable_survey = row['value']
        
        if mode == 'HELM_TOWAYPT' and variable_towaypoint is not None:
            return variable_towaypoint.replace('points=', '')
        elif mode == 'HELM_SURVEYING':
            ret = variable_survey.replace('points=', '')
            ret = ret.replace('points =', '')
            ret = ret.replace('pts=', '')
            ret = ret.replace('{', '')
            ret = ret.replace('}', '')
            ret = ret.replace(' ', '')
            
            desired_path = ret
            
        if desired_path is not None and variable_survey is not None:    
            return desired_path
        
    return None

def determine_desired_altitude(numerical_data, helm_start, helm_stop, timeStart):
    """
    Determine the desired altitude based on NAV_ALTITUDE data during the active helm period.
    If no helm_start/helm_stop provided or no data exists in that period, use the whole time range.
    """
    # Set defaults for the effective helm window
    helm_start_eff = helm_start if helm_start is not None else timeStart
    helm_stop_eff = helm_stop if helm_stop is not None else float('inf')  # Use infinity if no helm stop
    
    # Filter the numerical data for the active helm period
    filtered_data = numerical_data[
        (numerical_data['time'] >= helm_start_eff) & 
        (numerical_data['time'] <= helm_stop_eff) & 
        (numerical_data['variable'] == 'NAV_ALTITUDE')
    ]
    
    # If there are entries with DESIRED_ALTITUDE within the helm window, use them
    desired_alt_data = numerical_data[
        (numerical_data['time'] >= helm_start_eff) & 
        (numerical_data['time'] <= helm_stop_eff) & 
        (numerical_data['variable'] == 'DESIRED_ALTITUDE')
    ]
    
    if not desired_alt_data.empty:
        # Return the average of DESIRED_ALTITUDE values during the helm active period
        return desired_alt_data['value'].mean()
    
    # If no DESIRED_ALTITUDE, calculate the average NAV_ALTITUDE during the helm active period
    elif not filtered_data.empty:
        return filtered_data['value'].mean()
    
    
    # If still no data, try looking at the full dataset
    all_alt_data = numerical_data[numerical_data['variable'] == 'NAV_ALTITUDE']
    if not all_alt_data.empty:
        return all_alt_data['value'].mean()
    
    # Default fallback value if no altitude data is available
    return 100.0

# Function to read and process numerical CSV
def read_numerical_csv(file_path):
    # Read the CSV file with the correct separator and only numerical values
    numerical_data = pd.read_csv(file_path, delimiter=';', header=None, names=['time', 'variable', 'value'])
    
    # Ensure the 'value' column is numeric where appropriate
    numerical_data['time'] = pd.to_numeric(numerical_data['time'], errors='coerce')
    numerical_data['value'] = pd.to_numeric(numerical_data['value'], errors='coerce')
    
    # Drop any NaN values that might exist due to non-numeric data in value column
    numerical_data.dropna(subset=['time'], inplace=True)
    numerical_data.dropna(subset=['value'], inplace=True)
    
    return numerical_data

# Function to read categorical CSV (in case you need this data)
def read_categorical_csv(file_path):
    # Read the CSV file with the correct separator and only categorical values
    categorical_data = pd.read_csv(file_path, delimiter=';', header=None, names=['time', 'variable', 'value'])
    categorical_data['time'] = pd.to_numeric(categorical_data['time'], errors='coerce')
    categorical_data.dropna(subset=['time'], inplace=True)
    return categorical_data

# Function to read and process GCS telemetry CSV
def read_gcs_telemetry_csv(file_path):
    """Read GCS telemetry CSV file with NODE_REPORT and SURVEY_UPDATE entries."""
    try:
        # Read the CSV file with the correct separator
        data = pd.read_csv(file_path, delimiter=';', header=None, names=['time', 'variable', 'value'])
        
        # Ensure the 'time' column is numeric
        data['time'] = pd.to_numeric(data['time'], errors='coerce')
        data.dropna(subset=['time'], inplace=True)
        
        return data
    except Exception as e:
        print(f"Error reading GCS telemetry file: {e}")
        return None

# Function to parse NODE_REPORT messages
def parse_node_reports(data):
    """Parse NODE_REPORT entries from telemetry data and organize by vehicle name."""
    # Filter for node reports (both regular and vehicle-specific ones)
    node_reports = data[data['variable'].str.startswith('NODE_REPORT')]
    
    if node_reports.empty:
        print("No NODE_REPORT entries found in the data.")
        return None
    
    # Extract vehicle data from NODE_REPORT entries
    vehicles_data = {}
    
    for _, row in node_reports.iterrows():
        # Parse the NODE_REPORT value string to extract key=value pairs
        report_str = row['value']
        # Extract NAME first
        name_match = re.search(r'NAME=([^,]+)', report_str)
        if not name_match:
            continue
        
        vehicle_name = name_match.group(1)
        
        # Parse X, Y, ALTITUDE, COLOR
        x_match = re.search(r'X=(-?\d+\.?\d*)', report_str)
        y_match = re.search(r'Y=(-?\d+\.?\d*)', report_str)
        alt_match = re.search(r'ALTITUDE=(-?\d+\.?\d*)', report_str)
        color_match = re.search(r'COLOR=([^,]+)', report_str)
        
        if not (x_match and y_match):
            continue
        
        x = float(x_match.group(1))
        y = float(y_match.group(1))
        altitude = float(alt_match.group(1)) if alt_match else None
        color = color_match.group(1) if color_match else 'blue'
                
        # Add the data point to the vehicle's data
        if vehicle_name not in vehicles_data:
            vehicles_data[vehicle_name] = {
                'time': [row['time']],
                'x': [x],
                'y': [y],
                'altitude': [altitude] if altitude is not None else [],
                'color': color
            }
        else:
            vehicles_data[vehicle_name]['time'].append(row['time'])
            vehicles_data[vehicle_name]['x'].append(x)
            vehicles_data[vehicle_name]['y'].append(y)
            if altitude is not None:
                vehicles_data[vehicle_name]['altitude'].append(altitude)
    
    return vehicles_data

# Function to parse SURVEY_UPDATE messages
def parse_survey_updates(data):
    """Parse SURVEY_UPDATE entries from telemetry data and organize by vehicle name."""
    # Filter for survey updates
    survey_updates = data[data['variable'].str.startswith('SURVEY_UPDATE')]
    
    if survey_updates.empty:
        print("No SURVEY_UPDATE entries found in the data.")
        return None
    
    # Extract vehicle paths from SURVEY_UPDATE entries
    vehicles_paths = {}
    
    for _, row in survey_updates.iterrows():
        variable = row['variable']
        # Extract vehicle name from SURVEY_UPDATE_VEHICLENAME
        vehicle_name_match = re.search(r'SURVEY_UPDATE_([^;]+)', variable)
        vehicle_name = vehicle_name_match.group(1) if vehicle_name_match else "unknown"
        vehicle_name = vehicle_name.lower()
        
        # Extract points string
        points_str = row['value']
        points_match = re.search(r'pts=\{([^}]+)\}', points_str)
        if points_match:
            points = points_match.group(1)
            # Parse the points
            x_coords, y_coords = parse_desired_path(points)
            
            vehicles_paths[vehicle_name] = {
                'x': x_coords,
                'y': y_coords,
                'time': row['time']
            }
    
    return vehicles_paths

# Function to determine automatic helm stop time from node reports
def determine_helm_stop_from_reports(data, vehicle_name, helm_start):
    """
    Determine the automatic helm stop time by finding the first occurrence 
    of MODE=BHV_MODE@ACTIVE:RETURN after helm_start for a specific vehicle.
    
    Parameters:
    data (DataFrame): The telemetry data containing NODE_REPORT entries
    vehicle_name (str): The name of the vehicle to find helm stop for
    helm_start (float): The helm start time, return entries must be after this
    
    Returns:
    float or None: The time of the first RETURN mode after helm_start, or None if not found
    """
    # Filter for node reports for this vehicle
    vehicle_reports = data[data['variable'].str.contains('NODE_REPORT')]
    
    # If no helm start provided, we can't determine what's "after" it
    if helm_start is None:
        return None
    
    # Sort by time (just to be safe)
    vehicle_reports = vehicle_reports.sort_values(by='time')
    
    # Filter for reports after helm_start
    after_start = vehicle_reports[vehicle_reports['time'] >= helm_start]
    
    # Find the first occurrence of RETURN mode
    for _, row in after_start.iterrows():
        # Check if this report belongs to the specified vehicle
        if vehicle_name and not f"NAME={vehicle_name}" in row['value']:
            continue
            
        # Check if this report contains RETURN mode
        if 'MODE=BHV_MODE@ACTIVE:RETURN' in row['value']:
            return row['time']
    
    return None

# Function to determine helm stop time from DESIRED_SPEED=0.0
def determine_helm_stop_from_desired_speed(numerical_data, helm_start, timeStart, timeEnd):
    """
    Determine the automatic helm stop time by finding the earliest occurrence 
    of DESIRED_SPEED=0.0 after helm_start in numerical data.
    
    Parameters:
    numerical_data (DataFrame): The telemetry data containing variable/value pairs
    helm_start (float): The helm start time, entries must be after this
    timeStart (float): Default start time if helm_start is None
    timeEnd (float): Default end time if no stop time is found
    
    Returns:
    float: The time of the first DESIRED_SPEED=0.0 after helm_start, or timeEnd if not found
    """
    # If no helm start provided, we can't determine what's "after" it
    if helm_start is None:
        helm_start = timeStart
    
    # Filter for DESIRED_SPEED entries
    speed_data = numerical_data[(numerical_data['variable'] == 'DESIRED_SPEED') & 
                               (numerical_data['time'] >= helm_start)]
    
    # Sort by time (just to be safe)
    speed_data = speed_data.sort_values(by='time')
    
    # Find the first occurrence of DESIRED_SPEED=0.0
    zero_speed = speed_data[speed_data['value'] == 0.0]
    
    if not zero_speed.empty:
        # Return the earliest time when DESIRED_SPEED=0.0
        return zero_speed['time'].iloc[0]
    
    # If no zero speed found, return the end time
    return timeEnd

# Function to get automatic helm stops for all vehicles in the data
def get_automatic_helm_stops(data, helm_start):
    """
    Get automatic helm stop times for all vehicles in the data based on RETURN mode.
    
    Parameters:
    data (DataFrame): The telemetry data containing NODE_REPORT entries
    helm_start (float): The helm start time to filter events after
    
    Returns:
    dict: A dictionary mapping vehicle names to their helm stop times
    """
    # Parse node reports to get vehicle names
    vehicles_data = parse_node_reports(data)
    
    if not vehicles_data:
        return {}
        
    helm_stops = {}
    
    # Find helm stop for each vehicle
    for vehicle_name in vehicles_data.keys():
        stop_time = determine_helm_stop_from_reports(data, vehicle_name, helm_start)
        if stop_time is not None:
            helm_stops[vehicle_name] = stop_time
            
    return helm_stops

# Function to handle time series plots
def plot_time_series(numerical_data, timeStart, timeEnd, helm_start, helm_stop, var1, var2, title, ylabel, save_png=False, save_eps=False, file_path=None):
    # Filter data by time range
    filtered_data = numerical_data[(numerical_data['time'] >= timeStart) & (numerical_data['time'] <= timeEnd)]

    # Filter for the specific variables (var1 and var2)
    filtered_var1 = filtered_data[filtered_data['variable'] == var1].copy()
    filtered_var2 = filtered_data[filtered_data['variable'] == var2].copy()

    # Ensure both var1 and var2 are not empty after filtering
    if filtered_var1.empty or filtered_var2.empty:
        print(f"No valid data found for {var1} or {var2} in the specified time range.")
        return

    # Plotting
    plt.figure(figsize=(10, 5))
    label1 = var1 if var1 != 'DESIRED_HEADING' else 'DESIRED_COURSE'
    label2 = var2 if var2 != 'NAV_HEADING' else 'NAV_COURSE'
    plt.plot(filtered_var1['time'].to_numpy(), filtered_var1['value'].to_numpy(), label=label1, color='blue')
    plt.plot(filtered_var2['time'].to_numpy(), filtered_var2['value'].to_numpy(), label=label2, color='orange')

    # Mark helm start and end
    if helm_start is not None:
        plt.axvline(x=helm_start, color='green', linestyle='--', linewidth=1)
        plt.text(helm_start, plt.ylim()[0], 'Helm Start', color='green', verticalalignment='bottom', horizontalalignment='center')
        plt.axvspan(timeStart, helm_start, color='grey', alpha=0.3)
    if helm_stop is not None:
        plt.axvline(x=helm_stop, color='red', linestyle='--', linewidth=1)
        plt.text(helm_stop, plt.ylim()[0], 'Helm Stop', color='red', verticalalignment='bottom', horizontalalignment='center')
        plt.axvspan(helm_stop, timeEnd, color='grey', alpha=0.3)

    # Customize
    plt.title(title)
    plt.xlabel('Time (secs)')
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.xlim([timeStart, timeEnd])

    # if var1 contains 'ALTITUDE' then set the y-axis limits to min-30 and max+30
    if 'ALTITUDE' in var1 or 'ALTITUDE' in var2:
        plt.ylim([min(filtered_var1['value'].min(), filtered_var2['value'].min()) - 30,
                  max(filtered_var1['value'].max(), filtered_var2['value'].max()) + 30])

    # Add legend
    plt.legend(loc='best')

    # Save the plot
    if save_png and file_path:
        png_directory = os.path.join(os.path.dirname(file_path), 'png')
        ensure_directory_exists(png_directory)
        plt.savefig(os.path.join(png_directory, os.path.basename(file_path).replace('.csv', f'_{label1}_vs_{label2}.png')), format='png', dpi=300)
    
    if save_eps and file_path:
        eps_directory = os.path.join(os.path.dirname(file_path), 'eps')
        ensure_directory_exists(eps_directory)
        plt.savefig(os.path.join(eps_directory, os.path.basename(file_path).replace('.csv', f'_{label1}_vs_{label2}.eps')), format='eps', dpi=300)


    plt.tight_layout()
    plt.show()

# Function to plot 2D map using NAV_X and NAV_Y
def plot_2d_position(data, timeStart, timeEnd, helm_start, helm_stop, desired_path=None, fires_file=None, save_png=False, save_eps=False, file_path=None, sensor_position=None):
    # Compute effective helm window defaults
    helm_start_eff = helm_start if helm_start is not None else timeStart
    helm_stop_eff = helm_stop if helm_stop is not None else timeEnd
    # Filter data by time range
    filtered_data = data[(data['time'] >= timeStart) & (data['time'] <= timeEnd)]
    
    if filtered_data.empty:
        print("No data available in the specified time range.")
        return
    
    # Check if the required columns exist in the data
    required_columns = ['NAV_X', 'NAV_Y']
    data_columns = filtered_data['variable'].unique()
    missing_columns = [col for col in required_columns if col not in data_columns]
    
    if missing_columns:
        print(f"Error: Required columns missing from the data: {', '.join(missing_columns)}")
        print(f"Available columns: {', '.join(data_columns)}")
        return
    
    # Pivot the data to wide format for plotting
    filtered_data = filtered_data.pivot_table(index='time', columns='variable', values='value', aggfunc='first').reset_index()
    
    # Check if the pivoted dataframe is empty or missing required columns
    if filtered_data.empty or any(col not in filtered_data.columns for col in required_columns):
        print(f"Error: Unable to create plot. The pivoted dataframe is empty or missing required columns.")
        print(f"Available columns after pivot: {', '.join(filtered_data.columns)}")
        return
    
    # Interpolate 
    filtered_data = filtered_data.interpolate(method='linear')

    # Plotting the 2D map
    plt.figure(figsize=(8, 8))

    # Parse fires file if provided
    polygon_coords, fires = None, None
    if fires_file:
        polygon_coords, fires = parse_fires_file(fires_file)

    if not polygon_coords or not fires:
        print(f"No polygon or fire data available from the fires file {fires_file}.")

    # Plot polygon region if available
    if polygon_coords:
        x_poly, y_poly = polygon_coords
        # Close the polygon by adding the first point at the end
        x_closed = x_poly + [x_poly[0]]
        y_closed = y_poly + [y_poly[0]]
        plt.plot(x_closed, y_closed, color='purple', linestyle='-', linewidth=2, label='Region Polygon')
        plt.fill(x_closed, y_closed, alpha=0.1, color='purple')

    # Plot fires if available
    if fires:
        fire_x = [fire['x'] for fire in fires]
        fire_y = [fire['y'] for fire in fires]
        fire_names = [fire['name'] for fire in fires]
        
        plt.scatter(fire_x, fire_y, color='red', marker='D', s=80, label='Fires', zorder=5)
        
        # Add fire labels
        for i, name in enumerate(fire_names):
            plt.annotate(name, (fire_x[i], fire_y[i]), fontsize=8,
                         xytext=(5, 5), textcoords='offset points')

    # Grey out points before helm_start and after helm_stop
    if helm_start is not None:
        pre_helm_data = filtered_data[filtered_data['time'] < helm_start_eff]
        if not pre_helm_data.empty:
            plt.plot(pre_helm_data['NAV_X'].to_numpy(), pre_helm_data['NAV_Y'].to_numpy(), 
                    color='grey', alpha=0.3, linewidth=.5, label='Path Helm Inactive')
    
    if helm_stop is not None:
        post_helm_data = filtered_data[filtered_data['time'] > helm_stop_eff]
        if not post_helm_data.empty:
            plt.plot(post_helm_data['NAV_X'].to_numpy(), post_helm_data['NAV_Y'].to_numpy(), 
                    color='grey', alpha=0.3, linewidth=.5)

    # Plot the valid path in blue
    valid_data = filtered_data[(filtered_data['time'] >= helm_start_eff) & (filtered_data['time'] <= helm_stop_eff)]
    if not valid_data.empty:
        plt.plot(valid_data['NAV_X'].to_numpy(), valid_data['NAV_Y'].to_numpy(), 
                color='blue', label='Path Helm Active')

        # Mark start and end points if available
        plt.text(valid_data['NAV_X'].iloc[0], valid_data['NAV_Y'].iloc[0], 'Helm Start', 
                color='green', fontsize=12)
        plt.text(valid_data['NAV_X'].iloc[-1], valid_data['NAV_Y'].iloc[-1], 'Helm End', 
                color='red', fontsize=12)

        # Plot sensor radius circles
        # Determine position for sensor radius circles
        
        idx = len(valid_data) // 2 
        
        if (sensor_position is not None ) and 0 < sensor_position and sensor_position < 1:
            idx = round(len(valid_data) * sensor_position)        
            idx = min(idx, len(valid_data) - 1)
    
        
        sensor_x = valid_data['NAV_X'].iloc[idx]
        sensor_y = valid_data['NAV_Y'].iloc[idx]
        
        if(sensor_position!=-1):            
            # Plot inner sensor radius circle (diameter 50)
            inner_circle = plt.Circle((sensor_x, sensor_y), 25, color='blue', fill=True, alpha=0.15, 
                                    linestyle='--', linewidth=1, label='Inner Detect Range (20m)')
            plt.gca().add_patch(inner_circle)
            
            # Plot outer sensor radius circle (diameter 70)
            outer_circle = plt.Circle((sensor_x, sensor_y), 35, color='blue', fill=True, alpha=0.1, 
                                    linestyle='--', linewidth=1, label='Outer Detect Range (30m)')
            plt.gca().add_patch(outer_circle)
            
            # Add text annotation
            plt.text(sensor_x, sensor_y + 35, 'Detect Range', ha='center', color='blue', fontsize=10)

    # Plot the desired path if provided
    if desired_path:
        desired_x, desired_y = parse_desired_path(desired_path)
        plt.plot(desired_x, desired_y, color='black', linestyle='--', linewidth=2, label='Desired Path')
        plt.scatter(desired_x, desired_y, color='black', marker='o', s=DESIRED_PATH_VERTEX_SIZE, label='Desired Path Vertex')

    # Find the closest point to helm_start
    if helm_start is not None and not filtered_data.empty:
        closest_helm_start_idx = (filtered_data['time'] - helm_start).abs().idxmin()
        helm_start_data = filtered_data.loc[closest_helm_start_idx]
        plt.scatter(helm_start_data['NAV_X'], helm_start_data['NAV_Y'], 
                  color='green', marker='o', label='Helm Start', s=20)

    # Find the closest point to helm_stop
    if helm_stop is not None and not filtered_data.empty:
        closest_helm_stop_idx = (filtered_data['time'] - helm_stop).abs().idxmin()
        helm_stop_data = filtered_data.loc[closest_helm_stop_idx]
        plt.scatter(helm_stop_data['NAV_X'], helm_stop_data['NAV_Y'], 
                  color='red', marker='o', label='Helm Stop', s=20)

    # Customize plot
    plt.xlabel('NAV_X')
    plt.ylabel('NAV_Y')
    plt.title(f'2D Position Plot')
    plt.grid(True)
    plt.legend()

    # Adjust axis limits - include polygon and fires in the view if available
    if not filtered_data.empty:
        x_min = filtered_data['NAV_X'].min()
        x_max = filtered_data['NAV_X'].max()
        y_min = filtered_data['NAV_Y'].min()
        y_max = filtered_data['NAV_Y'].max()
        
        # Expand limits to include polygon if available
        if polygon_coords:
            x_poly, y_poly = polygon_coords
            x_min = min(x_min, min(x_poly))
            x_max = max(x_max, max(x_poly))
            y_min = min(y_min, min(y_poly))
            y_max = max(y_max, max(y_poly))
        
        # Expand limits to include fires if available
        if fires:
            fire_x = [fire['x'] for fire in fires]
            fire_y = [fire['y'] for fire in fires]
            if fire_x:
                x_min = min(x_min, min(fire_x))
                x_max = max(x_max, max(fire_x))
            if fire_y:
                y_min = min(y_min, min(fire_y))
                y_max = max(y_max, max(fire_y))
        
        # Add some padding
        padding = 0.1 * max(x_max - x_min, y_max - y_min)
        plt.xlim([x_min - padding, x_max + padding])
        plt.ylim([y_min - padding, y_max + padding])
    elif polygon_coords or fires:
        # If we don't have position data but we have polygon or fires, set limits based on those
        x_limits = []
        y_limits = []
        
        if polygon_coords:
            x_poly, y_poly = polygon_coords
            x_limits.extend([min(x_poly), max(x_poly)])
            y_limits.extend([min(y_poly), max(y_poly)])
        
        if fires:
            fire_x = [fire['x'] for fire in fires]
            fire_y = [fire['y'] for fire in fires]
            if fire_x:
                x_limits.extend([min(fire_x), max(fire_x)])
            if fire_y:
                y_limits.extend([min(fire_y), max(fire_y)])
                
        if x_limits and y_limits:
            x_min, x_max = min(x_limits), max(x_limits)
            y_min, y_max = min(y_limits), max(y_limits)
            
            # Add some padding
            padding = 0.1 * max(x_max - x_min, y_max - y_min)
            plt.xlim([x_min - padding, x_max + padding])
            plt.ylim([y_min - padding, y_max + padding])

    # Save the plot
    if save_png and file_path:
        png_directory = os.path.join(os.path.dirname(file_path), 'png')
        ensure_directory_exists(png_directory)
        plt.savefig(os.path.join(png_directory, os.path.basename(file_path).replace('.csv', '_2d_position.png')), format='png', dpi=300)

    if save_eps and file_path:
        eps_directory = os.path.join(os.path.dirname(file_path), 'eps')
        ensure_directory_exists(eps_directory)
        plt.savefig(os.path.join(eps_directory, os.path.basename(file_path).replace('.csv', '_2d_position.eps')), format='eps', dpi=300)

        pdf_directory = os.path.join(os.path.dirname(file_path), 'pdf')
        ensure_directory_exists(pdf_directory)
        plt.savefig(
            os.path.join(pdf_directory, os.path.basename(file_path).replace('.csv', '_2d_position.pdf')),
            format='pdf',
            dpi=300,
            transparent=True  
        )
        
    plt.tight_layout()
    plt.show()

# Function to plot 3D position using NAV_X, NAV_Y, and NAV_ALTITUDE
def plot_3d_position(data, timeStart, timeEnd, helm_start, helm_stop, desired_path=None, desired_altitude=None, save_png=False, save_eps=False, file_path=None):
    # Filter data by time range
    filtered_data = data[(data['time'] >= timeStart) & (data['time'] <= timeEnd)]
    
    if filtered_data.empty:
        print("No data available in the specified time range.")
        return
    
    # Check if the required columns exist in the data
    required_columns = ['NAV_X', 'NAV_Y', 'NAV_ALTITUDE']
    data_columns = filtered_data['variable'].unique()
    missing_columns = [col for col in required_columns if col not in data_columns]
    
    if missing_columns:
        print(f"Error: Required columns missing from the data: {', '.join(missing_columns)}")
        print(f"Available columns: {', '.join(data_columns)}")
        return
    
    # Pivot the data to wide format for plotting
    filtered_data = filtered_data.pivot_table(index='time', columns='variable', values='value', aggfunc='first').reset_index()
    
    # Check if the pivoted dataframe is empty or missing required columns
    if filtered_data.empty or any(col not in filtered_data.columns for col in required_columns):
        print(f"Error: Unable to create plot. The pivoted dataframe is empty or missing required columns.")
        print(f"Available columns after pivot: {', '.join(filtered_data.columns)}")
        return
    
    # Interpolate 
    filtered_data = filtered_data.interpolate(method='linear')

    # Compute effective helm window defaults
    helm_start_eff = helm_start if helm_start is not None else timeStart
    helm_stop_eff = helm_stop if helm_stop is not None else timeEnd

    # Plotting the 3D map
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Grey out points before helm_start and after helm_stop
    if helm_start is not None:
        pre_helm_data = filtered_data[filtered_data['time'] < helm_start_eff]
        if not pre_helm_data.empty:
            ax.plot(pre_helm_data['NAV_X'].to_numpy(), pre_helm_data['NAV_Y'].to_numpy(), 
                   pre_helm_data['NAV_ALTITUDE'].to_numpy(),
                   color='grey', alpha=0.3, linewidth=.5, label='Path Helm Inactive')
    
    if helm_stop is not None:
        post_helm_data = filtered_data[filtered_data['time'] > helm_stop_eff]
        if not post_helm_data.empty:
            ax.plot(post_helm_data['NAV_X'].to_numpy(), post_helm_data['NAV_Y'].to_numpy(), 
                   post_helm_data['NAV_ALTITUDE'].to_numpy(),
                   color='grey', alpha=0.3, linewidth=.5)

    # Plot the valid path in blue
    valid_data = filtered_data[(filtered_data['time'] >= helm_start_eff) & (filtered_data['time'] <= helm_stop_eff)]
    if not valid_data.empty:
        ax.plot(valid_data['NAV_X'].to_numpy(), valid_data['NAV_Y'].to_numpy(), 
               valid_data['NAV_ALTITUDE'].to_numpy(),
               color='blue', label='Path Helm Active')

        # Mark start and end points if available
        ax.scatter(valid_data['NAV_X'].iloc[0], valid_data['NAV_Y'].iloc[0], valid_data['NAV_ALTITUDE'].iloc[0],
                  color='green', marker='o', s=50)
        ax.text(valid_data['NAV_X'].iloc[0], valid_data['NAV_Y'].iloc[0], valid_data['NAV_ALTITUDE'].iloc[0], 
               'Helm Start', color='green', fontsize=12)
        
        ax.scatter(valid_data['NAV_X'].iloc[-1], valid_data['NAV_Y'].iloc[-1], valid_data['NAV_ALTITUDE'].iloc[-1],
                  color='red', marker='o', s=50)
        ax.text(valid_data['NAV_X'].iloc[-1], valid_data['NAV_Y'].iloc[-1], valid_data['NAV_ALTITUDE'].iloc[-1], 
               'Helm End', color='red', fontsize=12)

    # Plot the desired path if provided
    if desired_path:
        desired_x, desired_y = parse_desired_path(desired_path)
        if desired_altitude is None:
            desired_altitude = 100.0  # Default altitude if not provided
            
        desired_altitudes = [desired_altitude] * len(desired_x)
        ax.plot(desired_x, desired_y, desired_altitudes, 
               color='black', linestyle='--', linewidth=2, label='Desired Path')
        ax.scatter(desired_x, desired_y, desired_altitudes,
                  color='black', marker='o', s=DESIRED_PATH_VERTEX_SIZE)

    # Find the closest point to helm_start
    if helm_start is not None and not filtered_data.empty:
        closest_helm_start_idx = (filtered_data['time'] - helm_start).abs().idxmin()
        helm_start_data = filtered_data.loc[closest_helm_start_idx]
        ax.scatter(helm_start_data['NAV_X'], helm_start_data['NAV_Y'], helm_start_data['NAV_ALTITUDE'],
                  color='green', marker='o', s=100, label='Helm Start')

    # Find the closest point to helm_stop
    if helm_stop is not None and not filtered_data.empty:
        closest_helm_stop_idx = (filtered_data['time'] - helm_stop).abs().idxmin()
        helm_stop_data = filtered_data.loc[closest_helm_stop_idx]
        ax.scatter(helm_stop_data['NAV_X'], helm_stop_data['NAV_Y'], helm_stop_data['NAV_ALTITUDE'],
                  color='red', marker='o', s=100, label='Helm Stop')

    # Customize plot
    ax.set_xlabel('NAV_X')
    ax.set_ylabel('NAV_Y')
    ax.set_zlabel('Altitude [m]')
    ax.set_title(f'3D Position Plot')
    ax.legend()

    # Save the plot
    if save_png and file_path:
        png_directory = os.path.join(os.path.dirname(file_path), 'png')
        ensure_directory_exists(png_directory)
        plt.savefig(os.path.join(png_directory, os.path.basename(file_path).replace('.csv', '_3d_position.png')), format='png', dpi=300)

    if save_eps and file_path:
        eps_directory = os.path.join(os.path.dirname(file_path), 'eps')
        ensure_directory_exists(eps_directory)
        plt.savefig(os.path.join(eps_directory, os.path.basename(file_path).replace('.csv', '_3d_position.eps')), format='eps', dpi=300)

        pdf_directory = os.path.join(os.path.dirname(file_path), 'pdf')
        ensure_directory_exists(pdf_directory)
        plt.savefig(
            os.path.join(pdf_directory, os.path.basename(file_path).replace('.csv', '_3d_position.pdf')),
            format='pdf',
            dpi=300,
            transparent=True  
        )
    plt.tight_layout()
    plt.show()

# Function to plot multi-vehicle paths in 2D
def plot_multi_vehicle_2d(data, timeStart, timeEnd, helm_start, helm_stop, fires_file=None, save_png=False, save_eps=False, file_path=None, detect_position=0.5):
    """Plot 2D paths for multiple vehicles from NODE_REPORT data."""
    # Parse the node reports
    vehicles_data = parse_node_reports(data)
    if not vehicles_data:
        print("No valid vehicle data found for plotting.")
        return
    
    # Parse the survey updates (desired paths)
    vehicles_paths = parse_survey_updates(data)
    
    # Create the plot
    plt.figure(figsize=(10, 8))
    
    # Parse fires file if provided
    polygon_coords, fires = None, None
    if fires_file:
        polygon_coords, fires = parse_fires_file(fires_file)

    # Plot polygon region if available
    if polygon_coords:
        x_poly, y_poly = polygon_coords
        # Close the polygon by adding the first point at the end
        x_closed = x_poly + [x_poly[0]]
        y_closed = y_poly + [y_poly[0]]
        plt.plot(x_closed, y_closed, color='purple', linestyle='-', linewidth=2, label='Region Polygon')
        plt.fill(x_closed, y_closed, alpha=0.1, color='purple')

    # Plot fires if available
    if fires:
        fire_x = [fire['x'] for fire in fires]
        fire_y = [fire['y'] for fire in fires]
        fire_names = [fire['name'] for fire in fires]
        
        plt.scatter(fire_x, fire_y, color='red', marker='D', s=80, label='Fires', zorder=5)
        
        # Add fire labels
        for i, name in enumerate(fire_names):
            plt.annotate(name, (fire_x[i], fire_y[i]), fontsize=8,
                         xytext=(5, 5), textcoords='offset points')
    
    # Plot the desired paths first (so they appear behind the actual paths)
    DesiredPathLegend = False
    if vehicles_paths:
        for vehicle_name, path_data in vehicles_paths.items():
            # Get the vehicle color from vehicles_data if available, otherwise use black
            vehicle_color = 'black'
            if vehicle_name in vehicles_data:
                vehicle_color = vehicles_data[vehicle_name]['color']
            
            # Create a darker version of the vehicle color for the path
            try:
                # For named colors, we can use a darker variant
                darker_color = f"dark{vehicle_color}" if vehicle_color in plt.colormaps() else vehicle_color
            except:
                # If that fails, just use the original color
                darker_color = vehicle_color
            
            # print(f"Vehicle {vehicle_name} dark-color: {darker_color}")
            plt.plot(path_data['x'], path_data['y'], 
                     color="black", linestyle='--', linewidth=1.5, 
                     label=f'Desired Path' if not DesiredPathLegend else "")
            plt.scatter(path_data['x'], path_data['y'], 
                        color=darker_color, marker='o', s=DESIRED_PATH_VERTEX_SIZE)
            DesiredPathLegend = True

    VehicleInactiveLegend = False
    HelmStartStopLegend = False
    # Plot each vehicle's path
    for vehicle_name, vehicle_data in vehicles_data.items():
        # Convert lists to numpy arrays for easier manipulation
        times = pd.Series(vehicle_data['time'])
        x_coords = pd.Series(vehicle_data['x'])
        y_coords = pd.Series(vehicle_data['y'])
        color = vehicle_data['color']
        
        # Sort data by time
        df = pd.DataFrame({'time': times, 'x': x_coords, 'y': y_coords})
        df = df.sort_values(by='time')
        
        # Filter data by time range
        df = df[(df['time'] >= timeStart) & (df['time'] <= timeEnd)]
        
        if df.empty:
            print(f"No data points for vehicle {vehicle_name} in the specified time range.")
            continue
        
        # Compute effective helm window defaults
        helm_start_eff = helm_start if helm_start is not None else timeStart
        helm_stop_eff = helm_stop if helm_stop is not None else timeEnd
        
        # Grey out points before helm_start
        if helm_start is not None:
            pre_helm_data = df[df['time'] < helm_start_eff]
            if not pre_helm_data.empty:
                plt.plot(pre_helm_data['x'].to_numpy(), pre_helm_data['y'].to_numpy(), 
                        color='grey', alpha=0.3, linewidth=.5, label=f'Path Helm Inactive' if not VehicleInactiveLegend else "")
                VehicleInactiveLegend = True
                
        
        # Grey out points after helm_stop
        if helm_stop is not None:
            post_helm_data = df[df['time'] > helm_stop_eff]
            if not post_helm_data.empty:
                plt.plot(post_helm_data['x'].to_numpy(), post_helm_data['y'].to_numpy(), 
                        color='grey', alpha=0.3, linewidth=.5)
        
        # Plot the valid path with vehicle's color
        valid_data = df[(df['time'] >= helm_start_eff) & (df['time'] <= helm_stop_eff)]
        if not valid_data.empty:
            plt.plot(valid_data['x'].to_numpy(), valid_data['y'].to_numpy(), 
                    color=color, label=f'{vehicle_name}')
            
            # Mark start and end points
            plt.scatter(valid_data['x'].iloc[0], valid_data['y'].iloc[0], color='green', marker='o', s=30, label=f'Helm Start' if not HelmStartStopLegend else "")
            plt.text(valid_data['x'].iloc[0], valid_data['y'].iloc[0], f'{vehicle_name} Start', 
                    color='green', fontsize=8, verticalalignment='bottom')
            
            plt.scatter(valid_data['x'].iloc[-1], valid_data['y'].iloc[-1], color='red', marker='o', s=30, label=f'Helm Stop' if not HelmStartStopLegend else "")
            plt.text(valid_data['x'].iloc[-1], valid_data['y'].iloc[-1], f'{vehicle_name} End', 
                    color='red', fontsize=8, verticalalignment='bottom')
            HelmStartStopLegend = True
            
            # Add detect range visualization at the specified position along the valid path
            if 0 <= detect_position <= 1:
                idx = int(len(valid_data) * detect_position)
                idx = max(0, min(idx, len(valid_data) - 1))  # Ensure index is within bounds
            else:
                # Default to middle of path if detect_position is invalid
                idx = len(valid_data) // 2
                
            sensor_x = valid_data['x'].iloc[idx]
            sensor_y = valid_data['y'].iloc[idx]
            
            if(detect_position!=-1):                    
                # Plot inner sensor radius circle (diameter 50)
                inner_circle = plt.Circle((sensor_x, sensor_y), 25, color=color, fill=True, alpha=0.15, 
                                        linestyle='--', linewidth=1)
                plt.gca().add_patch(inner_circle)
                
                # Plot outer sensor radius circle (diameter 70)
                outer_circle = plt.Circle((sensor_x, sensor_y), 35, color=color, fill=True, alpha=0.1, 
                                        linestyle='--', linewidth=1)
                plt.gca().add_patch(outer_circle)
                
                # Add text annotation
                plt.text(sensor_x, sensor_y + 35, f'{vehicle_name} Range', ha='center', color=color, fontsize=8)
    
    # Customize the plot
    plt.xlabel('NAV_X')
    plt.ylabel('NAV_Y')
    plt.title('Multi-Vehicle 2D Position Plot')
    plt.grid(True)
    plt.legend(loc='best')
    
    # Set axis limits to include all data points
    all_x = []
    all_y = []
    
    # Add vehicle positions
    for vehicle_data in vehicles_data.values():
        all_x.extend(vehicle_data['x'])
        all_y.extend(vehicle_data['y'])
    
    # Add desired paths
    if vehicles_paths:
        for path_data in vehicles_paths.values():
            all_x.extend(path_data['x'])
            all_y.extend(path_data['y'])
    
    # Add polygon and fires
    if polygon_coords:
        all_x.extend(polygon_coords[0])
        all_y.extend(polygon_coords[1])
    
    if fires:
        all_x.extend([fire['x'] for fire in fires])
        all_y.extend([fire['y'] for fire in fires])
    
    if all_x and all_y:
        x_min, x_max = min(all_x), max(all_x)
        y_min, y_max = min(all_y), max(all_y)
        padding = 0.1 * max(x_max - x_min, y_max - y_min)
        plt.xlim([x_min - padding, x_max + padding])
        plt.ylim([y_min - padding, y_max + padding])
    
    # Save the plot
    if save_png and file_path:
        png_directory = os.path.join(os.path.dirname(file_path), 'png')
        ensure_directory_exists(png_directory)
        plt.savefig(os.path.join(png_directory, os.path.basename(file_path).replace('.csv', '_multi_vehicle_2d.png')), format='png', dpi=300)

    if save_eps and file_path:
        eps_directory = os.path.join(os.path.dirname(file_path), 'eps')
        ensure_directory_exists(eps_directory)
        plt.savefig(os.path.join(eps_directory, os.path.basename(file_path).replace('.csv', '_multi_vehicle_2d.eps')), format='eps', dpi=300)

        pdf_directory = os.path.join(os.path.dirname(file_path), 'pdf')
        ensure_directory_exists(pdf_directory)
        plt.savefig(
            os.path.join(pdf_directory, os.path.basename(file_path).replace('.csv', '_multi_vehicle_2d.pdf')),
            format='pdf',
            dpi=300,
            transparent=True  
        )

    plt.tight_layout()
    plt.show()

# Function to plot multi-vehicle paths in 3D
def plot_multi_vehicle_3d(data, timeStart, timeEnd, helm_start, helm_stop, save_png=False, save_eps=False, file_path=None, detect_position=0.5):
    """Plot 3D paths for multiple vehicles from NODE_REPORT data."""
    # Parse the node reports
    vehicles_data = parse_node_reports(data)
    if not vehicles_data:
        print("No valid vehicle data found for plotting.")
        return
    
    # Parse the survey updates (desired paths)
    vehicles_paths = parse_survey_updates(data)
    
    # Create the 3D plot
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the desired paths first (so they appear behind the actual paths)
    DesiredPathLegend = False
    if vehicles_paths:
        # Get default altitude for each vehicle from their data
        default_altitudes = {}
        
        # Compute effective helm window defaults
        helm_start_eff = helm_start if helm_start is not None else timeStart
        helm_stop_eff = helm_stop if helm_stop is not None else timeEnd
        
        for vehicle_name, vehicle_data in vehicles_data.items():
            if vehicle_data['altitude']:
                # Filter altitude data to only include values within helm active period
                helm_active_altitudes = []
                for i, t in enumerate(vehicle_data['time']):
                    if helm_start_eff <= t <= helm_stop_eff:
                        helm_active_altitudes.append(vehicle_data['altitude'][i])
                
                # Use the average altitude during helm active period as default for the desired path
                if helm_active_altitudes:
                    default_altitudes[vehicle_name] = sum(helm_active_altitudes) / len(helm_active_altitudes)
                    print(f"Vehicle {vehicle_name} active helm altitude average: {default_altitudes[vehicle_name]}")
                else:
                    # Fallback to using all altitude data if no data in helm active period
                    default_altitudes[vehicle_name] = sum(vehicle_data['altitude']) / len(vehicle_data['altitude'])
                    print(f"Vehicle {vehicle_name} full dataset altitude average: {default_altitudes[vehicle_name]}")
            else:
                default_altitudes[vehicle_name] = 100  # Default if no altitude data
                print(f"Vehicle {vehicle_name} using default altitude: 100")
        
        for vehicle_name, path_data in vehicles_paths.items():
            # Use the corresponding vehicle's average altitude for the desired path
            altitude = default_altitudes.get(vehicle_name, 100)
            desired_z = [altitude] * len(path_data['x'])
            
            # Get the vehicle color from vehicles_data if available, otherwise use black
            vehicle_color = 'black'
            if vehicle_name in vehicles_data:
                vehicle_color = vehicles_data[vehicle_name]['color']
            
            # Create a darker version of the vehicle color for the path
            try:
                # For named colors, we can use a darker variant
                darker_color = f"dark{vehicle_color}" if vehicle_color in plt.colormaps() else vehicle_color
            except:
                # If that fails, just use the original color
                darker_color = vehicle_color
            
            ax.plot(path_data['x'], path_data['y'], desired_z, 
                   color="black", linestyle='--', linewidth=1.5, 
                   label=f'Desired Path' if not DesiredPathLegend else "")
            ax.scatter(path_data['x'], path_data['y'], desired_z,
                      color=darker_color, marker='o', s=DESIRED_PATH_VERTEX_SIZE)
            DesiredPathLegend = True
    
    # Plot each vehicle's path
    VehicleInactiveLegend = False
    HelmStartStopLegend = False
    for vehicle_name, vehicle_data in vehicles_data.items():
        # Skip if no altitude data
        if not vehicle_data['altitude']:
            print(f"No altitude data for vehicle {vehicle_name}, skipping from 3D plot.")
            continue
        
        # Convert lists to numpy arrays for easier manipulation
        times = pd.Series(vehicle_data['time'])
        x_coords = pd.Series(vehicle_data['x'])
        y_coords = pd.Series(vehicle_data['y'])
        altitudes = pd.Series(vehicle_data['altitude'])
        color = vehicle_data['color']
        
        # Sort data by time
        df = pd.DataFrame({'time': times, 'x': x_coords, 'y': y_coords, 'altitude': altitudes})
        df = df.sort_values(by='time')
        
        # Filter data by time range
        df = df[(df['time'] >= timeStart) & (df['time'] <= timeEnd)]
        
        if df.empty:
            print(f"No data points for vehicle {vehicle_name} in the specified time range.")
            continue
        
        # Compute effective helm window defaults
        helm_start_eff = helm_start if helm_start is not None else timeStart
        helm_stop_eff = helm_stop if helm_stop is not None else timeEnd
        
        # Grey out points before helm_start
        if helm_start is not None:
            pre_helm_data = df[df['time'] < helm_start_eff]
            if not pre_helm_data.empty:
                ax.plot(pre_helm_data['x'].to_numpy(), pre_helm_data['y'].to_numpy(), pre_helm_data['altitude'].to_numpy(),
                       color='grey', alpha=0.3, linewidth=.5, label=f'Path Helm Inactive' if not VehicleInactiveLegend else "")
                VehicleInactiveLegend = True
        
        # Grey out points after helm_stop
        if helm_stop is not None:
            post_helm_data = df[df['time'] > helm_stop_eff]
            if not post_helm_data.empty:
                ax.plot(post_helm_data['x'].to_numpy(), post_helm_data['y'].to_numpy(), post_helm_data['altitude'].to_numpy(),
                       color='grey', alpha=0.3, linewidth=.5)
        
        # Plot the valid path with vehicle's color
        valid_data = df[(df['time'] >= helm_start_eff) & (df['time'] <= helm_stop_eff)]
        if not valid_data.empty:
            ax.plot(valid_data['x'].to_numpy(), valid_data['y'].to_numpy(), valid_data['altitude'].to_numpy(),
                   color=color, label=f'{vehicle_name}')
            
            # Mark start and end points
            ax.scatter(valid_data['x'].iloc[0], valid_data['y'].iloc[0], valid_data['altitude'].iloc[0], 
                      color='green', marker='o', s=30, label=f'Helm Start' if not HelmStartStopLegend else "")
            ax.text(valid_data['x'].iloc[0], valid_data['y'].iloc[0], valid_data['altitude'].iloc[0],
                   f'{vehicle_name} Start', color='green', fontsize=8)
            
            ax.scatter(valid_data['x'].iloc[-1], valid_data['y'].iloc[-1], valid_data['altitude'].iloc[-1],
                      color='red', marker='o', s=30, label=f'Helm Stop' if not HelmStartStopLegend else "")
            ax.text(valid_data['x'].iloc[-1], valid_data['y'].iloc[-1], valid_data['altitude'].iloc[-1],
                   f'{vehicle_name} End', color='red', fontsize=8)
            HelmStartStopLegend = True
    
    # Customize the plot
    ax.set_xlabel('NAV_X')
    ax.set_ylabel('NAV_Y')
    ax.set_zlabel('Altitude [m]')
    ax.set_title('Multi-Vehicle 3D Position Plot')
    ax.legend(loc='best')
    
    # Save the plot
    if save_png and file_path:
        png_directory = os.path.join(os.path.dirname(file_path), 'png')
        ensure_directory_exists(png_directory)
        plt.savefig(os.path.join(png_directory, os.path.basename(file_path).replace('.csv', '_multi_vehicle_3d.png')), format='png', dpi=300)

    if save_eps and file_path:
        eps_directory = os.path.join(os.path.dirname(file_path), 'eps')
        ensure_directory_exists(eps_directory)
        plt.savefig(os.path.join(eps_directory, os.path.basename(file_path).replace('.csv', '_multi_vehicle_3d.eps')), format='eps', dpi=300)

        pdf_directory = os.path.join(os.path.dirname(file_path), 'pdf')
        ensure_directory_exists(pdf_directory)
        plt.savefig(
            os.path.join(pdf_directory, os.path.basename(file_path).replace('.csv', '_multi_vehicle_3d.pdf')),
            format='pdf',
            dpi=300,
            transparent=True  
        )

    plt.tight_layout()
    plt.show()

# Function to plot multi-vehicle paths in 2D with vehicle-specific helm stops
def plot_multi_vehicle_2d_with_auto_stops(data, timeStart, timeEnd, helm_start, vehicle_helm_stops, fires_file=None, save_png=False, save_eps=False, file_path=None, detect_position=0.5):
    """
    Plot 2D paths for multiple vehicles from NODE_REPORT data with vehicle-specific helm stop times.
    
    Parameters:
    data (DataFrame): The telemetry data containing NODE_REPORT entries
    timeStart, timeEnd (float): The overall time range for plotting
    helm_start (float): The global helm start time
    vehicle_helm_stops (dict): A dictionary mapping vehicle names to their specific helm stop times
    """
    # Parse the node reports
    vehicles_data = parse_node_reports(data)
    if not vehicles_data:
        print("No valid vehicle data found for plotting.")
        return
    
    # Parse the survey updates (desired paths)
    vehicles_paths = parse_survey_updates(data)
    
    # Create the plot
    plt.figure(figsize=(10, 8))
    
    # Parse fires file if provided
    polygon_coords, fires = None, None
    if fires_file:
        polygon_coords, fires = parse_fires_file(fires_file)

    # Plot polygon region if available
    if polygon_coords:
        x_poly, y_poly = polygon_coords
        # Close the polygon by adding the first point at the end
        x_closed = x_poly + [x_poly[0]]
        y_closed = y_poly + [y_poly[0]]
        plt.plot(x_closed, y_closed, color='purple', linestyle='-', linewidth=2, label='Region Polygon')
        plt.fill(x_closed, y_closed, alpha=0.1, color='purple')

    # Plot fires if available
    if fires:
        fire_x = [fire['x'] for fire in fires]
        fire_y = [fire['y'] for fire in fires]
        fire_names = [fire['name'] for fire in fires]
        
        plt.scatter(fire_x, fire_y, color='red', marker='D', s=80, label='Fires', zorder=5)
        
        # Add fire labels
        for i, name in enumerate(fire_names):
            plt.annotate(name, (fire_x[i], fire_y[i]), fontsize=8,
                         xytext=(5, 5), textcoords='offset points')
    
    # Plot the desired paths first (so they appear behind the actual paths)
    DesiredPathLegend = False
    if vehicles_paths:
        for vehicle_name, path_data in vehicles_paths.items():
            # Get the vehicle color from vehicles_data if available, otherwise use black
            vehicle_color = 'black'
            if vehicle_name in vehicles_data:
                vehicle_color = vehicles_data[vehicle_name]['color']
            
            # Create a darker version of the vehicle color for the path
            try:
                # For named colors, we can use a darker variant
                darker_color = f"dark{vehicle_color}" if vehicle_color in plt.colormaps() else vehicle_color
            except:
                # If that fails, just use the original color
                darker_color = vehicle_color
            
            plt.plot(path_data['x'], path_data['y'], 
                     color="black", linestyle='--', linewidth=1.5, 
                     label=f'Desired Path' if not DesiredPathLegend else "")
            plt.scatter(path_data['x'], path_data['y'], 
                        color=darker_color, marker='o', s=DESIRED_PATH_VERTEX_SIZE)
            DesiredPathLegend = True
    
    # Plot each vehicle's path with its specific helm stop time
    VehicleInactiveLegend = False
    HelmStartStopLegend = False
    for vehicle_name, vehicle_data in vehicles_data.items():
        # Convert lists to numpy arrays for easier manipulation
        times = pd.Series(vehicle_data['time'])
        x_coords = pd.Series(vehicle_data['x'])
        y_coords = pd.Series(vehicle_data['y'])
        color = vehicle_data['color']
        
        # Sort data by time
        df = pd.DataFrame({'time': times, 'x': x_coords, 'y': y_coords})
        df = df.sort_values(by='time')
        
        # Filter data by time range
        df = df[(df['time'] >= timeStart) & (df['time'] <= timeEnd)]
        
        if df.empty:
            print(f"No data points for vehicle {vehicle_name} in the specified time range.")
            continue
        
        # Get vehicle-specific helm stop time or use overall timeEnd if not available
        helm_stop = vehicle_helm_stops.get(vehicle_name, timeEnd)
        print(f"Vehicle {vehicle_name} helm stop time: {helm_stop}")
        
        # Compute effective helm window defaults
        helm_start_eff = helm_start if helm_start is not None else timeStart
        helm_stop_eff = helm_stop if helm_stop is not None else timeEnd
        
        # Grey out points before helm_start
        if helm_start is not None:
            pre_helm_data = df[df['time'] < helm_start_eff]
            if not pre_helm_data.empty:
                plt.plot(pre_helm_data['x'].to_numpy(), pre_helm_data['y'].to_numpy(), 
                        color='grey', alpha=0.3, linewidth=.5, label=f'Path Helm Inactive' if not VehicleInactiveLegend else "")
                VehicleInactiveLegend = True
        
        # Grey out points after vehicle-specific helm_stop
        post_helm_data = df[df['time'] > helm_stop_eff]
        if not post_helm_data.empty:
            plt.plot(post_helm_data['x'].to_numpy(), post_helm_data['y'].to_numpy(), 
                    color='grey', alpha=0.3, linewidth=.5)
        
        # Plot the valid path with vehicle's color
        valid_data = df[(df['time'] >= helm_start_eff) & (df['time'] <= helm_stop_eff)]
        if not valid_data.empty:
            plt.plot(valid_data['x'].to_numpy(), valid_data['y'].to_numpy(), 
                    color=color, label=f'{vehicle_name}')
            
            # Mark start and end points
            plt.scatter(valid_data['x'].iloc[0], valid_data['y'].iloc[0], color='green', marker='o', s=30, label=f'Helm Start' if not HelmStartStopLegend else "")
            plt.text(valid_data['x'].iloc[0], valid_data['y'].iloc[0], f'{vehicle_name} Start', 
                    color='green', fontsize=8, verticalalignment='bottom')
            
            plt.scatter(valid_data['x'].iloc[-1], valid_data['y'].iloc[-1], color='red', marker='o', s=30, label=f'Helm Stop' if not HelmStartStopLegend else "")
            plt.text(valid_data['x'].iloc[-1], valid_data['y'].iloc[-1], f'{vehicle_name} End', 
                    color='red', fontsize=8, verticalalignment='bottom')
            HelmStartStopLegend = True
            
            # Add detect range visualization at the specified position along the valid path
            if 0 <= detect_position <= 1:
                idx = int(len(valid_data) * detect_position)
                idx = max(0, min(idx, len(valid_data) - 1))  # Ensure index is within bounds
            else:
                # Default to middle of path if detect_position is invalid
                idx = len(valid_data) // 2
                
            sensor_x = valid_data['x'].iloc[idx]
            sensor_y = valid_data['y'].iloc[idx]
            
            if(detect_position!=-1):   
                # Plot inner sensor radius circle (diameter 50)
                inner_circle = plt.Circle((sensor_x, sensor_y), 25, color=color, fill=True, alpha=0.15, 
                                        linestyle='--', linewidth=1)
                plt.gca().add_patch(inner_circle)
                
                # Plot outer sensor radius circle (diameter 70)
                outer_circle = plt.Circle((sensor_x, sensor_y), 35, color=color, fill=True, alpha=0.1, 
                                        linestyle='--', linewidth=1)
                plt.gca().add_patch(outer_circle)
                
                # Add text annotation
                plt.text(sensor_x, sensor_y + 35, f'{vehicle_name} Range', ha='center', color=color, fontsize=8)
    
    # Customize the plot
    plt.xlabel('NAV_X')
    plt.ylabel('NAV_Y')
    plt.title('Multi-Vehicle 2D Position Plot')
    plt.grid(True)
    plt.legend(loc='best')
    
    # Set axis limits to include all data points
    all_x = []
    all_y = []
    
    # Add vehicle positions
    for vehicle_data in vehicles_data.values():
        all_x.extend(vehicle_data['x'])
        all_y.extend(vehicle_data['y'])
    
    # Add desired paths
    if vehicles_paths:
        for path_data in vehicles_paths.values():
            all_x.extend(path_data['x'])
            all_y.extend(path_data['y'])
    
    # Add polygon and fires
    if polygon_coords:
        all_x.extend(polygon_coords[0])
        all_y.extend(polygon_coords[1])
    
    if fires:
        all_x.extend([fire['x'] for fire in fires])
        all_y.extend([fire['y'] for fire in fires])
    
    if all_x and all_y:
        x_min, x_max = min(all_x), max(all_x)
        y_min, y_max = min(all_y), max(all_y)
        padding = 0.1 * max(x_max - x_min, y_max - y_min)
        plt.xlim([x_min - padding, x_max + padding])
        plt.ylim([y_min - padding, y_max + padding])
    
    # Save the plot
    if save_png and file_path:
        png_directory = os.path.join(os.path.dirname(file_path), 'png')
        ensure_directory_exists(png_directory)
        plt.savefig(os.path.join(png_directory, os.path.basename(file_path).replace('.csv', '_multi_vehicle_2d_auto.png')), format='png', dpi=300)

    if save_eps and file_path:
        eps_directory = os.path.join(os.path.dirname(file_path), 'eps')
        ensure_directory_exists(eps_directory)
        plt.savefig(os.path.join(eps_directory, os.path.basename(file_path).replace('.csv', '_multi_vehicle_2d_auto.eps')), format='eps', dpi=300)


        pdf_directory = os.path.join(os.path.dirname(file_path), 'pdf')
        ensure_directory_exists(pdf_directory)
        plt.savefig(
            os.path.join(pdf_directory, os.path.basename(file_path).replace('.csv', '_multi_vehicle_2d_auto.pdf')),
            format='pdf',
            dpi=300,
            transparent=True  
        )
        
    plt.tight_layout()
    plt.show()

# Function to plot multi-vehicle paths in 3D with vehicle-specific helm stops
def plot_multi_vehicle_3d_with_auto_stops(data, timeStart, timeEnd, helm_start, vehicle_helm_stops, save_png=False, save_eps=False, file_path=None, detect_position=0.5):
    """
    Plot 3D paths for multiple vehicles from NODE_REPORT data with vehicle-specific helm stop times.
    
    Parameters:
    data (DataFrame): The telemetry data containing NODE_REPORT entries
    timeStart, timeEnd (float): The overall time range for plotting
    helm_start (float): The global helm start time
    vehicle_helm_stops (dict): A dictionary mapping vehicle names to their specific helm stop times
    """
    # Parse the node reports
    vehicles_data = parse_node_reports(data)
    if not vehicles_data:
        print("No valid vehicle data found for plotting.")
        return
    
    # Parse the survey updates (desired paths)
    vehicles_paths = parse_survey_updates(data)
    
    # Create the 3D plot
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the desired paths first (so they appear behind the actual paths)
    DesiredPathLegend = False
    if vehicles_paths:
        # Get default altitude for each vehicle from their data during their active helm periods
        default_altitudes = {}
        
        for vehicle_name, vehicle_data in vehicles_data.items():
            if not vehicle_data['altitude']:
                continue
                
            # Get vehicle-specific helm stop time
            helm_stop = vehicle_helm_stops.get(vehicle_name, timeEnd)
            
            # Compute effective helm window defaults for this vehicle
            helm_start_eff = helm_start if helm_start is not None else timeStart
            helm_stop_eff = helm_stop if helm_stop is not None else timeEnd
            
            # Filter altitude data to only include values within helm active period
            helm_active_altitudes = []
            for i, t in enumerate(vehicle_data['time']):
                if helm_start_eff <= t <= helm_stop_eff:
                    helm_active_altitudes.append(vehicle_data['altitude'][i])
            
            # Use the average altitude during helm active period as default for the desired path
            if helm_active_altitudes:
                default_altitudes[vehicle_name] = sum(helm_active_altitudes) / len(helm_active_altitudes)
                print(f"Vehicle {vehicle_name} active helm altitude average: {default_altitudes[vehicle_name]}")
            else:
                # Fallback to using all altitude data if no data in helm active period
                default_altitudes[vehicle_name] = sum(vehicle_data['altitude']) / len(vehicle_data['altitude'])
                print(f"Vehicle {vehicle_name} full dataset altitude average: {default_altitudes[vehicle_name]}")
        
        # Fill in any missing default altitudes with a fallback value
        for vehicle_name in vehicles_paths.keys():
            if vehicle_name not in default_altitudes:
                default_altitudes[vehicle_name] = 100  # Default if no altitude data
                print(f"Vehicle {vehicle_name} using default altitude: 100")
        
        for vehicle_name, path_data in vehicles_paths.items():
            # Use the corresponding vehicle's average altitude for the desired path
            altitude = default_altitudes.get(vehicle_name, 100)
            desired_z = [altitude] * len(path_data['x'])
            
            # Get the vehicle color from vehicles_data if available, otherwise use black
            vehicle_color = 'black'
            if vehicle_name in vehicles_data:
                vehicle_color = vehicles_data[vehicle_name]['color']
            
            # Create a darker version of the vehicle color for the path
            try:
                # For named colors, we can use a darker variant
                darker_color = f"dark{vehicle_color}" if vehicle_color in plt.colormaps() else vehicle_color
            except:
                # If that fails, just use the original color
                darker_color = vehicle_color
            
            ax.plot(path_data['x'], path_data['y'], desired_z, 
                   color="black", linestyle='--', linewidth=1.5, 
                   label=f'Desired Path' if not DesiredPathLegend else "")
            ax.scatter(path_data['x'], path_data['y'], desired_z,
                      color=darker_color, marker='o', s=DESIRED_PATH_VERTEX_SIZE)
            DesiredPathLegend = True
    
    # Plot each vehicle's path with its specific helm stop time
    VehicleInactiveLegend = False
    HelmStartStopLegend = False
    for vehicle_name, vehicle_data in vehicles_data.items():
        # Skip if no altitude data
        if not vehicle_data['altitude']:
            print(f"No altitude data for vehicle {vehicle_name}, skipping from 3D plot.")
            continue
        
        # Convert lists to numpy arrays for easier manipulation
        times = pd.Series(vehicle_data['time'])
        x_coords = pd.Series(vehicle_data['x'])
        y_coords = pd.Series(vehicle_data['y'])
        altitudes = pd.Series(vehicle_data['altitude'])
        color = vehicle_data['color']
        
        # Sort data by time
        df = pd.DataFrame({'time': times, 'x': x_coords, 'y': y_coords, 'altitude': altitudes})
        df = df.sort_values(by='time')
        
        # Filter data by time range
        df = df[(df['time'] >= timeStart) & (df['time'] <= timeEnd)]
        
        if df.empty:
            print(f"No data points for vehicle {vehicle_name} in the specified time range.")
            continue
        
        # Get vehicle-specific helm stop time or use overall timeEnd if not available
        helm_stop = vehicle_helm_stops.get(vehicle_name, timeEnd)
        print(f"Vehicle {vehicle_name} helm stop time: {helm_stop}")
        
        # Compute effective helm window defaults
        helm_start_eff = helm_start if helm_start is not None else timeStart
        helm_stop_eff = helm_stop if helm_stop is not None else timeEnd
        
        # Grey out points before helm_start
        if helm_start is not None:
            pre_helm_data = df[df['time'] < helm_start_eff]
            if not pre_helm_data.empty:
                ax.plot(pre_helm_data['x'].to_numpy(), pre_helm_data['y'].to_numpy(), pre_helm_data['altitude'].to_numpy(),
                       color='grey', alpha=0.3, linewidth=.5, label=f'Path Helm Inactive' if not VehicleInactiveLegend else "")
                VehicleInactiveLegend = True
        
        # Grey out points after helm_stop
        if helm_stop is not None:
            post_helm_data = df[df['time'] > helm_stop_eff]
            if not post_helm_data.empty:
                ax.plot(post_helm_data['x'].to_numpy(), post_helm_data['y'].to_numpy(), post_helm_data['altitude'].to_numpy(),
                   color='grey', alpha=0.3, linewidth=.5)
        
        # Plot the valid path with vehicle's color
        valid_data = df[(df['time'] >= helm_start_eff) & (df['time'] <= helm_stop_eff)]
        if not valid_data.empty:
            ax.plot(valid_data['x'].to_numpy(), valid_data['y'].to_numpy(), valid_data['altitude'].to_numpy(),
                   color=color, label=f'{vehicle_name}')
            
            # Mark start and end points
            ax.scatter(valid_data['x'].iloc[0], valid_data['y'].iloc[0], valid_data['altitude'].iloc[0], 
                      color='green', marker='o', s=30, label=f'Helm Start' if not HelmStartStopLegend else "")
            ax.text(valid_data['x'].iloc[0], valid_data['y'].iloc[0], valid_data['altitude'].iloc[0],
                   f'{vehicle_name} Start', color='green', fontsize=8)
            
            ax.scatter(valid_data['x'].iloc[-1], valid_data['y'].iloc[-1], valid_data['altitude'].iloc[-1],
                      color='red', marker='o', s=30,label=f'Helm Start' if not HelmStartStopLegend else "")
            ax.text(valid_data['x'].iloc[-1], valid_data['y'].iloc[-1], valid_data['altitude'].iloc[-1],
                   f'{vehicle_name} End', color='red', fontsize=8)
    
    # Customize the plot
    ax.set_xlabel('NAV_X')
    ax.set_ylabel('NAV_Y')
    ax.set_zlabel('Altitude [m]')
    ax.set_title('Multi-Vehicle 3D Position Plot ')
    ax.legend(loc='best')
    
    ax.view_init(elev=20, azim=135)
    
    # Save the plot
    if save_png and file_path:
        png_directory = os.path.join(os.path.dirname(file_path), 'png')
        ensure_directory_exists(png_directory)
        plt.savefig(os.path.join(png_directory, os.path.basename(file_path).replace('.csv', '_multi_vehicle_3d_auto.png')), format='png', dpi=300)

    if save_eps and file_path:
        eps_directory = os.path.join(os.path.dirname(file_path), 'eps')
        ensure_directory_exists(eps_directory)
        plt.savefig(os.path.join(eps_directory, os.path.basename(file_path).replace('.csv', '_multi_vehicle_3d_auto.eps')), format='eps', dpi=300)

        pdf_directory = os.path.join(os.path.dirname(file_path), 'pdf')
        ensure_directory_exists(pdf_directory)
        plt.savefig(
            os.path.join(pdf_directory, os.path.basename(file_path).replace('.csv', '_multi_vehicle_3d_auto.pdf')),
            format='pdf',
            dpi=300,
            transparent=True  
        )
        
    plt.tight_layout()
    plt.show()

# Main function to handle command-line arguments
def main():
    parser = argparse.ArgumentParser(description='Plot CSV data for variables or position with helm start/stop lines.')
    
    # Required arguments for the CSV files
    parser.add_argument('numerical_file_path', type=str, help='Path to the numerical CSV file.')
    parser.add_argument('--categorical_file_path', '--cfp', type=str, help='Path to the categorical CSV file (optional).')
    parser.add_argument('--gcs_telemetry_path', '--gtp', type=str, help='Path to the GCS telemetry CSV file with NODE_REPORT data.')

    # Optional arguments for time start/end, helm start/stop
    parser.add_argument('--timeStart', type=float, default=None, help='Start time for the plot. Default is min time in the data.')
    parser.add_argument('--timeEnd', type=float, default=None, help='End time for the plot. Default is max time in the data.')
    parser.add_argument('--helmStart', type=float, default=None, help='Time when helm starts. Default is None.')
    parser.add_argument('--helmStop', type=float, default=None, 
                        help='Time when helm stops. If not provided, it will be automatically determined based on RETURN mode.')
    parser.add_argument('--autoHelmStop', action='store_true', 
                        help='Automatically determine helmet stop time based on MODE=BHV_MODE@ACTIVE:RETURN in Node Reports.')

    # Plot types
    parser.add_argument('--plotAll', action='store_true', help='Plot all available plots.')
    parser.add_argument('--plotHeading', action='store_true', help='Plot Desired Heading vs. NAV Heading. (NOTE: This is actually course over ground)')
    parser.add_argument('--plotSpeed', action='store_true', help='Plot Desired Speed vs. NAV Speed.')
    parser.add_argument('--plotAltitude', action='store_true', help='Plot Desired Altitude vs. NAV Altitude.')
    parser.add_argument('--plot2D', action='store_true', help='Plot 2D position map using NAV_X and NAV_Y.')
    parser.add_argument('--plot3D', action='store_true', help='Plot 3D position map using NAV_X, NAV_Y, and NAV_ALTITUDE.')
    parser.add_argument('--plotMultiVehicle2D','--plotGCS2D', action='store_true', help='Plot 2D position map for multiple vehicles from GCS telemetry.')
    parser.add_argument('--plotMultiVehicle3D','--plotGCS3D', action='store_true', help='Plot 3D position map for multiple vehicles from GCS telemetry.')

    # Flags for saving as PNG and/or EPS
    parser.add_argument('-s', action='store_true', help='Save plot as both PNG and EPS.')
    parser.add_argument('-sp', action='store_true', help='Save plot as PNG only.')
    parser.add_argument('-se', action='store_true', help='Save plot as EPS only.')

    # Optional argument for constant target altitude if DESIRED_ALTITUDE is missing
    parser.add_argument('--default_altitude', type=float, default=None, help='Default altitude to use if DESIRED_ALTITUDE is missing.')
    
    # Optional argument for desired path in 2D and 3D plots
    parser.add_argument('--desiredPath', type=str, help='Desired path as colon-separated coordinates (e.g., "10,20:15,25:20,30").')
    
    # argument for fires file
    parser.add_argument('--fireFile', type=str, help='Path to a fires file containing region polygon and fire coordinates.')

    # Optional argument for sensor position
    parser.add_argument('--sensorPosition', '--sp', type=float, default=0.5, 
                        help='Position of the detection range visualization as a fraction of the path (0 to 1). Default is 0.5 (middle of path).')
    
    # Optional argument for detect position in multi-vehicle plots
    parser.add_argument('--detectPosition', '--dp', type=float, default=0.5,
                        help='Position of the detection range visualization as a fraction of the path (0 to 1). Default is 0.5 (middle of path).')
    
    # Parse the arguments
    args = parser.parse_args()

    # Load the numerical CSV file
    numerical_data = None
    if args.numerical_file_path and args.numerical_file_path != "None" and not args.gcs_telemetry_path:
        numerical_data = read_numerical_csv(args.numerical_file_path)

    # Load the categorical CSV file if provided
    categorical_data = None
    if args.categorical_file_path:
        categorical_data = read_categorical_csv(args.categorical_file_path)
    
    # Load GCS telemetry CSV if provided
    gcs_telemetry_data = None
    if args.gcs_telemetry_path:
        gcs_telemetry_data = read_gcs_telemetry_csv(args.gcs_telemetry_path)

    # Determine the time range
    if numerical_data is not None:
        timeStart = args.timeStart if args.timeStart is not None else numerical_data['time'].min()
        timeEnd = args.timeEnd if args.timeEnd is not None else numerical_data['time'].max()
    elif gcs_telemetry_data is not None:
        timeStart = args.timeStart if args.timeStart is not None else gcs_telemetry_data['time'].min()
        timeEnd = args.timeEnd if args.timeEnd is not None else gcs_telemetry_data['time'].max()
    else:
        timeStart = args.timeStart if args.timeStart is not None else 0
        timeEnd = args.timeEnd if args.timeEnd is not None else 1000

    # Determine helm stop time
    helm_stop = args.helmStop
    
    # Auto-determine helm stop time if requested and no explicit helm stop provided
    if (args.autoHelmStop or helm_stop is None) and args.helmStart is not None and gcs_telemetry_data is not None:
        auto_helm_stops = get_automatic_helm_stops(gcs_telemetry_data, args.helmStart)
        
        if auto_helm_stops:
            # For single-vehicle plots, use the first vehicle's helm stop
            if not args.helmStop and (args.plot2D or args.plot3D or args.plotHeading or args.plotSpeed or args.plotAltitude):
                first_vehicle = next(iter(auto_helm_stops))
                helm_stop = auto_helm_stops[first_vehicle]
                print(f"Auto-detected helm stop time for {first_vehicle}: {helm_stop}")
            
            # For multi-vehicle plots, we'll use vehicle-specific helm stops in the plotting functions
            if args.plotMultiVehicle2D or args.plotMultiVehicle3D:
                print(f"Auto-detected helm stop times: {auto_helm_stops}")
    
    # Determine helm stop time from DESIRED_SPEED=0.0 if numerical data is available
    if helm_stop is None and numerical_data is not None and args.helmStart is not None:
        helm_stop = determine_helm_stop_from_desired_speed(numerical_data, args.helmStart, timeStart, timeEnd)
        print(f"Helm stop time determined from DESIRED_SPEED=0.0: {helm_stop}")

    # Determine save options
    save_png = args.s or args.sp
    save_eps = args.s or args.se

    # If plotAll is specified, enable all plot options
    if args.plotAll:
        args.plotHeading = True
        args.plotSpeed = True
        args.plotAltitude = True
        args.plot2D = True
        args.plot3D = True
        if args.gcs_telemetry_path:
            args.plotMultiVehicle2D = True
            args.plotMultiVehicle3D = True

    # Determine the desired path if not provided
    desired_path = args.desiredPath
    if not desired_path and categorical_data is not None:
        desired_path = determine_target_path(categorical_data, args.helmStart, timeStart)

    # Determine the desired altitude if default is not given
    desired_altitude = args.default_altitude
    if desired_altitude is None and numerical_data is not None:
        desired_altitude = determine_desired_altitude(numerical_data, args.helmStart, helm_stop, timeStart)

    print(f"Default Average desired altitude: {desired_altitude}")
    
    # Plot Heading
    if args.plotHeading and numerical_data is not None:
        if 'DESIRED_HEADING' in numerical_data['variable'].unique() and 'NAV_HEADING' in numerical_data['variable'].unique():
            plot_time_series(
                numerical_data, timeStart, timeEnd, args.helmStart, helm_stop,
                'DESIRED_HEADING', 'NAV_HEADING',
                'Course over ground', 'COG [deg]',
                save_png, save_eps, args.numerical_file_path
            )
        else:
            print("Desired Heading or NAV Heading data not available.")

    # Plot Speed
    if args.plotSpeed and numerical_data is not None:
        if 'DESIRED_SPEED' in numerical_data['variable'].unique() and 'NAV_SPEED' in numerical_data['variable'].unique():
            plot_time_series(
                numerical_data, timeStart, timeEnd, args.helmStart, helm_stop,
                'DESIRED_SPEED', 'NAV_SPEED',
                'Airspeed over Time', 'Airspeed [m/s]',
                save_png, save_eps, args.numerical_file_path
            )
        else:
            print("Desired Speed or NAV Speed data not available.")

    # Plot Altitude
    if args.plotAltitude and numerical_data is not None:
        if 'NAV_ALTITUDE' in numerical_data['variable'].unique():
            if 'DESIRED_ALTITUDE' in numerical_data['variable'].unique():
                plot_time_series(
                    numerical_data, timeStart, timeEnd, args.helmStart, helm_stop,
                    'DESIRED_ALTITUDE', 'NAV_ALTITUDE',
                    'Altitude over Time', 'Altitude AGL [m]',
                    save_png, save_eps, args.numerical_file_path
                )
            elif desired_altitude is not None:
                numerical_data['value'] = numerical_data.apply(lambda row: desired_altitude if row['variable'] == 'DESIRED_ALTITUDE' else row['value'], axis=1)
                # Add rows for 'DESIRED_ALTITUDE' 
                # Create new rows with default altitude value
                new_rows = numerical_data[numerical_data['variable'] == 'NAV_ALTITUDE'].copy()
                new_rows['variable'] = 'DESIRED_ALTITUDE'
                new_rows['value'] = desired_altitude

                # Append new rows to the DataFrame
                numerical_data = pd.concat([numerical_data, new_rows], ignore_index=True)
                plot_time_series(
                    numerical_data, timeStart, timeEnd, args.helmStart, helm_stop,
                    'DESIRED_ALTITUDE', 'NAV_ALTITUDE',
                    'Altitude over Time', 'Altitude AGL[m]',
                    save_png, save_eps, args.numerical_file_path
                )
            else:
                print("Desired Altitude data is not available, and no default altitude was provided.")
        else:
            print("NAV Altitude data not available.")

    # Plot 2D Position
    if args.plot2D and numerical_data is not None:
        if 'NAV_X' in numerical_data['variable'].unique() and 'NAV_Y' in numerical_data['variable'].unique():
            plot_2d_position(
                numerical_data, timeStart, timeEnd, args.helmStart, helm_stop,
                desired_path=desired_path,
                fires_file=args.fireFile,
                save_png=save_png, save_eps=save_eps, file_path=args.numerical_file_path,
                sensor_position=args.sensorPosition
            )
        else:
            print("NAV_X or NAV_Y data not available for 2D position plot.")

    # Plot 3D Position
    if args.plot3D and numerical_data is not None:
        if 'NAV_X' in numerical_data['variable'].unique() and 'NAV_Y' in numerical_data['variable'].unique() and 'NAV_ALTITUDE' in numerical_data['variable'].unique():
            plot_3d_position(
                numerical_data, timeStart, timeEnd, args.helmStart, helm_stop,
                desired_path=desired_path,
                desired_altitude=desired_altitude,
                save_png=save_png, save_eps=save_eps, file_path=args.numerical_file_path
            )
        else:
            print("NAV_X, NAV_Y, or NAV_ALTITUDE data not available for 3D position plot.")
    
    # Plot Multi-Vehicle 2D Position
    if args.plotMultiVehicle2D and gcs_telemetry_data is not None:
        # For multi-vehicle plots, auto-determine helm stops for each vehicle if needed
        vehicle_helm_stops = None
        if (args.autoHelmStop or helm_stop is None) and args.helmStart is not None:
            vehicle_helm_stops = get_automatic_helm_stops(gcs_telemetry_data, args.helmStart)
            
        # Use global helm stop if vehicle-specific ones are not available
        if not vehicle_helm_stops:
            plot_multi_vehicle_2d(
                gcs_telemetry_data, timeStart, timeEnd, args.helmStart, helm_stop,
                fires_file=args.fireFile,
                save_png=save_png, save_eps=save_eps, file_path=args.gcs_telemetry_path,
                detect_position=args.detectPosition or args.sensorPosition
            )
        else:
            # Call a modified version of plot_multi_vehicle_2d that uses vehicle-specific helm stops
            plot_multi_vehicle_2d_with_auto_stops(
                gcs_telemetry_data, timeStart, timeEnd, args.helmStart, vehicle_helm_stops,
                fires_file=args.fireFile,
                save_png=save_png, save_eps=save_eps, file_path=args.gcs_telemetry_path,
                detect_position=args.detectPosition or args.sensorPosition
            )
    
    # Plot Multi-Vehicle 3D Position
    if args.plotMultiVehicle3D and gcs_telemetry_data is not None:
        # For multi-vehicle plots, auto-determine helm stops for each vehicle if needed
        vehicle_helm_stops = None
        if (args.autoHelmStop or helm_stop is None) and args.helmStart is not None:
            vehicle_helm_stops = get_automatic_helm_stops(gcs_telemetry_data, args.helmStart)
            
        # Use global helm stop if vehicle-specific ones are not available
        if not vehicle_helm_stops:
            plot_multi_vehicle_3d(
                gcs_telemetry_data, timeStart, timeEnd, args.helmStart, helm_stop,
                save_png=save_png, save_eps=save_eps, file_path=args.gcs_telemetry_path,
                detect_position=args.detectPosition or args.sensorPosition
            )
        else:
            # Call a modified version of plot_multi_vehicle_3d that uses vehicle-specific helm stops
            plot_multi_vehicle_3d_with_auto_stops(
                gcs_telemetry_data, timeStart, timeEnd, args.helmStart, vehicle_helm_stops,
                save_png=save_png, save_eps=save_eps, file_path=args.gcs_telemetry_path,
                detect_position=args.detectPosition or args.sensorPosition
            )

if __name__ == '__main__':
    main()

