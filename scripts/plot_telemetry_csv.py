import argparse
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

import re


FILTERTIME_OFFSETT = -150

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


# Function to automatically determine the target path from the data
def determine_target_path(categorical_data, helm_start, timeStart):
    if categorical_data is None:
        return None

    # Filter the categorical data after helm_start or timeStart
    start_time = helm_start if helm_start is not None else timeStart
    filtered_categorical = categorical_data[categorical_data['time'] >= start_time]
    
    #sort filtered data after time
    filtered_categorical = filtered_categorical.sort_values(by='time')
    
    
    mode=None
    variable_towaypoint = None
    variable_survey = None
    
    print(filtered_categorical)
    # Iterate through the categorical data to find the last relevant mode and corresponding update
    for _, row in filtered_categorical.iterrows():

        if row['variable'] == 'MODE':
            mode = row['value']
        elif row['variable'] == 'TOWAYPT_UPDATE':
            variable_towaypoint = row['value']
        elif row['variable'] == 'SURVEY_UPDATE':
            variable_survey = row['value']
        
        if mode == 'HELM_TOWAYPT' and variable_towaypoint is not None:
            return variable_towaypoint.replace('points=', '')
        elif mode == 'HELM_SURVEYING' and variable_survey is not None:
            return variable_survey.replace('points=', '')
        
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

    # if var1 =='DESIRED_SPEED' :
    #     # replace all the values of desired speed with 12
    #     filtered_var1['value'] = 11.7
    
    # Ensure both var1 and var2 are not empty after filtering
    if filtered_var1.empty or filtered_var2.empty:
        print(f"No valid data found for {var1} or {var2} in the specified time range.")
        return

    # Plotting
    plt.figure(figsize=(10, 5))
    plt.plot(filtered_var1['time'], filtered_var1['value'], label=var1, color='blue')
    plt.plot(filtered_var2['time'], filtered_var2['value'], label=var2, color='orange')

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
def plot_2d_position(data, timeStart, timeEnd, helm_start, helm_stop, desired_path=None, save_png=False, save_eps=False, file_path=None):
       
    
    # Filter data by time range
    filtered_data = data[(data['time'] >= timeStart) & (data['time'] <= timeEnd)]

    filtered_data = filtered_data.pivot_table(index='time', columns='variable', values='value', aggfunc='first').reset_index()
    
    # Interpolate 
    filtered_data = filtered_data.interpolate(method='linear')

    
    # Plotting the 2D map
    plt.figure(figsize=(8, 8))

    # Grey out points before helm_start and after helm_stop
    if helm_start is not None:
        pre_helm_data = filtered_data[filtered_data['time'] < helm_start]
        plt.plot(pre_helm_data['NAV_X'], pre_helm_data['NAV_Y'], color='grey', alpha=0.3, linewidth=.5, label='Path Helm Inactive')
    if helm_stop is not None:
        post_helm_data = filtered_data[filtered_data['time'] > helm_stop]
        plt.plot(post_helm_data['NAV_X'], post_helm_data['NAV_Y'], color='grey', alpha=0.3, linewidth=.5)

    # Plot the valid path in blue
    valid_data = filtered_data[(filtered_data['time'] >= helm_start) & (filtered_data['time'] <= helm_stop)]
    plt.plot(valid_data['NAV_X'], valid_data['NAV_Y'], color='blue', label='Path Helm Active')

    # Mark start and end points
    plt.text(valid_data['NAV_X'].iloc[0], valid_data['NAV_Y'].iloc[0], 'Helm Start', color='green', fontsize=12)
    plt.text(valid_data['NAV_X'].iloc[-1], valid_data['NAV_Y'].iloc[-1], 'Helm End', color='red', fontsize=12)

    # Plot the desired path if provided
    if desired_path:
        desired_x, desired_y = parse_desired_path(desired_path)
        plt.plot(desired_x, desired_y, color='black', linestyle='--', linewidth=2, label='Desired Path')
        plt.scatter(desired_x, desired_y, color='black', marker='o', s=50, label='Desired Path Vertex')

    # Find the closest point to helm_start
    if helm_start is not None:
        closest_helm_start_idx = (filtered_data['time'] - helm_start).abs().idxmin()
        helm_start_data = filtered_data.loc[closest_helm_start_idx]
        plt.scatter(helm_start_data['NAV_X'], helm_start_data['NAV_Y'], color='green', marker='o', label='Helm Start', s=20)

    # Find the closest point to helm_stop
    if helm_stop is not None:
        closest_helm_stop_idx = (filtered_data['time'] - helm_stop).abs().idxmin()
        helm_stop_data = filtered_data.loc[closest_helm_stop_idx]
        plt.scatter(helm_stop_data['NAV_X'], helm_stop_data['NAV_Y'], color='red', marker='o', label='Helm Stop', s=20)

    # Customize plot
    plt.xlabel('NAV_X')
    plt.ylabel('NAV_Y')
    plt.title(f'2D Position Plot')
    plt.grid(True)
    plt.legend()

    plt.xlim([filtered_data['NAV_X'].min(), filtered_data['NAV_X'].max()])
    plt.ylim([filtered_data['NAV_Y'].min(), filtered_data['NAV_Y'].max()])

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
        ax.plot(pre_helm_data['NAV_X'], pre_helm_data['NAV_Y'], pre_helm_data['NAV_ALTITUDE'], color='grey', alpha=0.3, linewidth=.5, label='Path Helm Inactive')
    if helm_stop is not None:
        post_helm_data = filtered_data[filtered_data['time'] > helm_stop]
        ax.plot(post_helm_data['NAV_X'], post_helm_data['NAV_Y'], post_helm_data['NAV_ALTITUDE'], color='grey', alpha=0.3, linewidth=.5)

    # Plot the valid path in blue
    valid_data = filtered_data[(filtered_data['time'] >= helm_start) & (filtered_data['time'] <= helm_stop)]
    ax.plot(valid_data['NAV_X'], valid_data['NAV_Y'], valid_data['NAV_ALTITUDE'], color='blue', label='Path Helm Active')

    # Mark start and end points
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
        ax.scatter(desired_x, desired_y, desired_z, color='black', marker='o', s=50, label='Desired Path Vertex')


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
                save_png=save_png, save_eps=save_eps, file_path=args.numerical_file_path
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

