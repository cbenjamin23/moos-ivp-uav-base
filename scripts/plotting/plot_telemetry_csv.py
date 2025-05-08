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
    
    print(f"filtered categorical:\n {filtered_categorical}")
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

def determine_desired_altitude(numerical_data, helm_start, timeStart):
    
     # Filter the categorical data after helm_start or timeStart
    start_time = helm_start if helm_start is not None else timeStart
    filtered_data = numerical_data[numerical_data['time'] >= start_time & (numerical_data['variable'] == 'DESIRED_ALTITUDE')]
    
    # If there are any entries, return the first DESIRED_ALTITUDE found
    if not filtered_data.empty:
        return filtered_data.iloc[0]['value']
    
    return None

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
    plt.plot(filtered_var1['time'].to_numpy(), filtered_var1['value'].to_numpy(), label=var1, color='blue')
    plt.plot(filtered_var2['time'].to_numpy(), filtered_var2['value'].to_numpy(), label=var2, color='orange')

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
        plt.savefig(os.path.join(png_directory, os.path.basename(file_path).replace('.csv', f'_{var1}_vs_{var2}.png')), format='png', dpi=300)
    if save_eps and file_path:
        eps_directory = os.path.join(os.path.dirname(file_path), 'eps')
        ensure_directory_exists(eps_directory)
        plt.savefig(os.path.join(eps_directory, os.path.basename(file_path).replace('.csv', f'_{var1}_vs_{var2}.eps')), format='eps', dpi=300)

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
            
        # Plot inner sensor radius circle (diameter 50)
        inner_circle = plt.Circle((sensor_x, sensor_y), 25, color='blue', fill=True, alpha=0.15, 
                                 linestyle='--', linewidth=1, label='Inner Detect Range (20m)')
        plt.gca().add_patch(inner_circle)
        
        # Plot outer sensor radius circle (diameter 70)
        outer_circle = plt.Circle((sensor_x, sensor_y), 35, color='blue', fill=True, alpha=0.05, 
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

    plt.tight_layout()
    plt.show()

# Function to plot 3D map using NAV_X, NAV_Y, and NAV_ALTITUDE
def plot_3d_position(data, timeStart, timeEnd, helm_start, helm_stop, desired_path=None, desired_altitude=None, save_png=False, save_eps=False, file_path=None):
    
    # Filter data by time range
    filtered_data = data[(data['time'] >= timeStart) & (data['time'] <= timeEnd)]

    filtered_data = filtered_data.pivot_table(index='time', columns='variable', values='value', aggfunc='first').reset_index()
    
    # Interpolate 
    filtered_data = filtered_data.interpolate(method='linear')
    


    # Create 3D plot
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Grey out points before helm_start and after helm_stop
    if helm_start is not None:
        pre_helm_data = filtered_data[filtered_data['time'] < helm_start]
        ax.plot(pre_helm_data['NAV_X'].to_numpy(), pre_helm_data['NAV_Y'].to_numpy(), pre_helm_data['NAV_ALTITUDE'].to_numpy(), color='grey', alpha=0.3, linewidth=.5, label='Path Helm Inactive')
    if helm_stop is not None:
        post_helm_data = filtered_data[filtered_data['time'] > helm_stop]
        ax.plot(post_helm_data['NAV_X'].to_numpy(), post_helm_data['NAV_Y'].to_numpy(), post_helm_data['NAV_ALTITUDE'].to_numpy(), color='grey', alpha=0.3, linewidth=.5)

    # Plot the valid path in blue and mark start/end if data exists
    valid_data = filtered_data[(filtered_data['time'] >= helm_start) & (filtered_data['time'] <= helm_stop)]
    ax.plot(valid_data['NAV_X'].to_numpy(), valid_data['NAV_Y'].to_numpy(), valid_data['NAV_ALTITUDE'].to_numpy(), color='blue', label='Path Helm Active')
    # Mark start and end points if available
    if not valid_data.empty:
        ax.text(valid_data['NAV_X'].iloc[0], valid_data['NAV_Y'].iloc[0], valid_data['NAV_ALTITUDE'].iloc[0], 'Helm Start', color='green')
        ax.text(valid_data['NAV_X'].iloc[-1], valid_data['NAV_Y'].iloc[-1], valid_data['NAV_ALTITUDE'].iloc[-1], 'Helm End', color='red')

    # Plot the desired path if provided
    if desired_path and desired_altitude is not None:
        desired_x, desired_y = parse_desired_path(desired_path)
        # desired z should be the same as desired_altitude
            
        # Assuming desired_altitude is already a list or Series with corresponding altitudes
        if isinstance(desired_altitude, (list, pd.Series)) and len(desired_altitude) == len(desired_x):
            desired_z = desired_altitude
        else:
            # If desired_altitude is a constant, replicate it across all points
            desired_z = [desired_altitude] * len(desired_x)
        ax.plot(desired_x, desired_y, desired_z, color='black', linestyle='--', linewidth=2, label='Desired Path')
        ax.scatter(desired_x, desired_y, desired_z, color='black', marker='o', s=DESIRED_PATH_VERTEX_SIZE, label='Desired Path Vertex')


    # Find the closest point to helm_start
    if helm_start is not None:
        closest_helm_start_idx = (filtered_data['time'] - helm_start).abs().idxmin()
        helm_start_data = filtered_data.loc[closest_helm_start_idx]
        ax.scatter(helm_start_data['NAV_X'], helm_start_data['NAV_Y'], helm_start_data['NAV_ALTITUDE'], color='green', marker='o', label='Helm Start', s=20)

    # Find the closest point to helm_stop
    if helm_stop is not None:
        closest_helm_stop_idx = (filtered_data['time'] - helm_stop).abs().idxmin()
        helm_stop_data = filtered_data.loc[closest_helm_stop_idx]
        ax.scatter(helm_stop_data['NAV_X'], helm_stop_data['NAV_Y'], helm_stop_data['NAV_ALTITUDE'], color='red', marker='o', label='Helm Stop', s=20)

    # Customize plot
    ax.set_xlabel('NAV_X')
    ax.set_ylabel('NAV_Y')
    ax.set_zlabel('NAV_ALTITUDE')
    ax.set_title('3D Position Plot')
    ax.legend()

    # Adjust axis limits
    ax.set_xlim([filtered_data['NAV_X'].min(), filtered_data['NAV_X'].max()])
    ax.set_ylim([filtered_data['NAV_Y'].min(), filtered_data['NAV_Y'].max()])
    ax.set_zlim([filtered_data['NAV_ALTITUDE'].min() - 30, filtered_data['NAV_ALTITUDE'].max() + 30])

    # Save the plot
    if save_png and file_path:
        png_directory = os.path.join(os.path.dirname(file_path), 'png')
        ensure_directory_exists(png_directory)
        plt.savefig(os.path.join(png_directory, os.path.basename(file_path).replace('.csv', '_3d_position.png')), format='png', dpi=300)

    if save_eps and file_path:
        eps_directory = os.path.join(os.path.dirname(file_path), 'eps')
        ensure_directory_exists(eps_directory)
        plt.savefig(os.path.join(eps_directory, os.path.basename(file_path).replace('.csv', '_3d_position.eps')), format='eps', dpi=300)

    plt.tight_layout()
    plt.show()

# Main function to handle command-line arguments
def main():
    parser = argparse.ArgumentParser(description='Plot CSV data for variables or position with helm start/stop lines.')
    
    # Required arguments for the CSV files
    parser.add_argument('numerical_file_path', type=str, help='Path to the numerical CSV file.')
    parser.add_argument('--categorical_file_path', '--cfp', type=str, help='Path to the categorical CSV file (optional).')

    # Optional arguments for time start/end, helm start/stop
    parser.add_argument('--timeStart', type=float, default=None, help='Start time for the plot. Default is min time in the data.')
    parser.add_argument('--timeEnd', type=float, default=None, help='End time for the plot. Default is max time in the data.')
    parser.add_argument('--helmStart', type=float, default=None, help='Time when helm starts. Default is None.')
    parser.add_argument('--helmStop', type=float, default=None, help='Time when helm stops. Default is None.')

    # Plot types
    parser.add_argument('--plotAll', action='store_true', help='Plot all available plots.')
    parser.add_argument('--plotHeading', action='store_true', help='Plot Desired Heading vs. NAV Heading.')
    parser.add_argument('--plotSpeed', action='store_true', help='Plot Desired Speed vs. NAV Speed.')
    parser.add_argument('--plotAltitude', action='store_true', help='Plot Desired Altitude vs. NAV Altitude.')
    parser.add_argument('--plot2D', action='store_true', help='Plot 2D position map using NAV_X and NAV_Y.')
    parser.add_argument('--plot3D', action='store_true', help='Plot 3D position map using NAV_X, NAV_Y, and NAV_ALTITUDE.')

    # Flags for saving as PNG and/or EPS
    parser.add_argument('-s', action='store_true', help='Save plot as both PNG and EPS.')
    parser.add_argument('-sp', action='store_true', help='Save plot as PNG only.')
    parser.add_argument('-se', action='store_true', help='Save plot as EPS only.')

    # Optional argument for constant target altitude if DESIRED_ALTITUDE is missing
    parser.add_argument('--default_altitude', type=float, default=100, help='Default altitude to use if DESIRED_ALTITUDE is missing.')
    
    # Optional argument for desired path in 2D and 3D plots
    parser.add_argument('--desiredPath', type=str, help='Desired path as colon-separated coordinates (e.g., "10,20:15,25:20,30").')
    
    # argument for fires file
    parser.add_argument('--fireFile', type=str, help='Path to a fires file containing region polygon and fire coordinates.')

    # Optional argument for sensor position
    parser.add_argument('--sensorPosition', '--sp', type=float, default=None, help='Sensor position as a fraction of the path (0 to 1).')
    
    # Parse the arguments
    args = parser.parse_args()

    # Load the numerical CSV file
    numerical_data = read_numerical_csv(args.numerical_file_path)

    # Load the categorical CSV file if provided
    categorical_data = None
    if args.categorical_file_path:
        categorical_data = read_categorical_csv(args.categorical_file_path)

    # Determine the time range
    timeStart = args.timeStart if args.timeStart is not None else numerical_data['time'].min()
    timeEnd = args.timeEnd if args.timeEnd is not None else numerical_data['time'].max()

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

    # Determine the desired path if not provided
    desired_path = args.desiredPath
    if not desired_path and categorical_data is not None:
        desired_path = determine_target_path(categorical_data, args.helmStart, timeStart)

    print("Desired Path: ", desired_path)
    

    # Determine the desired altitude if default is not given
    desired_altitude = args.default_altitude
    if desired_altitude is None:
        desired_altitude = determine_desired_altitude(numerical_data, args.helmStart, timeStart)

    # Plot Heading
    if args.plotHeading:
        if 'DESIRED_HEADING' in numerical_data['variable'].unique() and 'NAV_HEADING' in numerical_data['variable'].unique():
            plot_time_series(
                numerical_data, timeStart, timeEnd, args.helmStart, args.helmStop,
                'DESIRED_HEADING', 'NAV_HEADING',
                'Heading over Time', 'Heading [deg]',
                save_png, save_eps, args.numerical_file_path
            )
        else:
            print("Desired Heading or NAV Heading data not available.")

    # Plot Speed
    if args.plotSpeed:
        if 'DESIRED_SPEED' in numerical_data['variable'].unique() and 'NAV_SPEED' in numerical_data['variable'].unique():
            plot_time_series(
                numerical_data, timeStart, timeEnd, args.helmStart, args.helmStop,
                'DESIRED_SPEED', 'NAV_SPEED',
                'Speed over Time', 'Speed [m/s]',
                save_png, save_eps, args.numerical_file_path
            )
        else:
            print("Desired Speed or NAV Speed data not available.")

    # Plot Altitude
    if args.plotAltitude:
        if 'NAV_ALTITUDE' in numerical_data['variable'].unique():
            if 'DESIRED_ALTITUDE' in numerical_data['variable'].unique():
                plot_time_series(
                    numerical_data, timeStart, timeEnd, args.helmStart, args.helmStop,
                    'DESIRED_ALTITUDE', 'NAV_ALTITUDE',
                    'Altitude over Time', 'Altitude AGL [m]',
                    save_png, save_eps, args.numerical_file_path
                )
            elif args.default_altitude is not None:
                numerical_data['value'] = numerical_data.apply(lambda row: args.default_altitude if row['variable'] == 'DESIRED_ALTITUDE' else row['value'], axis=1)
                # Add rows for 'DESIRED_ALTITUDE' 
                # Create new rows with default altitude value
                new_rows = numerical_data[numerical_data['variable'] == 'NAV_ALTITUDE'].copy()
                new_rows['variable'] = 'DESIRED_ALTITUDE'
                new_rows['value'] = args.default_altitude

                # Append new rows to the DataFrame
                numerical_data = pd.concat([numerical_data, new_rows], ignore_index=True)
                plot_time_series(
                    numerical_data, timeStart, timeEnd, args.helmStart, args.helmStop,
                    'DESIRED_ALTITUDE', 'NAV_ALTITUDE',
                    'Altitude over Time', 'Altitude AGL[m]',
                    save_png, save_eps, args.numerical_file_path
                )
            else:
                print("Desired Altitude data is not available, and no default altitude was provided.")
        else:
            print("NAV Altitude data not available.")

    # Plot 2D Position
    if args.plot2D:
        if 'NAV_X' in numerical_data['variable'].unique() and 'NAV_Y' in numerical_data['variable'].unique():
            plot_2d_position(
                numerical_data, timeStart, timeEnd, args.helmStart, args.helmStop,
                desired_path=desired_path,
                fires_file=args.fireFile,
                save_png=save_png, save_eps=save_eps, file_path=args.numerical_file_path,
                sensor_position=args.sensorPosition
            )
        else:
            print("NAV_X or NAV_Y data not available for 2D position plot.")

    # Plot 3D Position
    if args.plot3D:
        if 'NAV_X' in numerical_data['variable'].unique() and 'NAV_Y' in numerical_data['variable'].unique() and 'NAV_ALTITUDE' in numerical_data['variable'].unique():
            plot_3d_position(
                numerical_data, timeStart, timeEnd, args.helmStart, args.helmStop,
                desired_path=desired_path,
                desired_altitude=desired_altitude,
                save_png=save_png, save_eps=save_eps, file_path=args.numerical_file_path
            )
        else:
            print("NAV_X, NAV_Y, or NAV_ALTITUDE data not available for 3D position plot.")

if __name__ == '__main__':
    main()

