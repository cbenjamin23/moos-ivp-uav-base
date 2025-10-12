import os
import re
import csv
from pathlib import Path
from collections import Counter, defaultdict

import argparse


# Define a function to parse a single mission score file
def parse_score_file(filepath):
    with open(filepath, 'r') as file:
        content = file.read()

    # Use regex to extract values from the content
    patterns = {
        'TotalScore': r"Total Score:\s*([\d.]+)",
        'Completeness': r"Completeness Score:\s*([\d.]+)",
        'TimeEfficiency': r"Time Efficiency Score:\s*([\d.]+)",
        'CoverageScore': r"Coverage Score:\s*([\d.]+)",
        'RedundantPenalty': r"Redundant Detection Penalty:\s*(-?[\d.]+)",
        'LatestDetTime': r"Latest Detection Time:\s*([\d.]+)",
        'AvgDetTime': r"Average Detection Time:\s*([\d.]+)",
        'MedDetTime': r"Median Detection Time:\s*([\d.]+)",
        'FiresDetected': r"Fires Detected:\s*(\d+)",
        'TotalFires': r"Fires Detected:\s*\d+\s*/\s*(\d+)",
        'TotalDetections': r"Total Detections:\s*(\d+)",
        'AreaCoverage': r"Area Coverage:\s*([\d.]+)",
        'Algorithm': r"Algorithm:\s*(\w+)",
        'IgnoredRegions': r"Ignored Regions:\s*(\d+)",
        'Deadline': r"Deadline:\s*([\d.]+)",
        'DroneCount': r"Drone Count:\s*(\d+)"
    }

    data = {}
    for key, pattern in patterns.items():
        match = re.search(pattern, content)
        data[key] = match.group(1) if match else None

    # Convert numeric fields
    numeric_fields = [
        'TotalScore', 'Completeness', 'TimeEfficiency', 'CoverageScore',
        'RedundantPenalty', 'LatestDetTime', 'AvgDetTime', 'MedDetTime',
        'FiresDetected', 'TotalFires', 'TotalDetections', 'AreaCoverage',
        'IgnoredRegions', 'Deadline', 'DroneCount'
    ]
    for field in numeric_fields:
        if data[field] is not None:
            data[field] = float(data[field]) if '.' in data[field] else int(data[field])
        else:
            data[field] = 0  # Default value if missing

    return data

def generate_mission_score_csv(input_folder="sim", output_folder=None, output_csv_name="mission_scores", ignored_regions=None):
    """
    Generate a CSV file with mission scores from individual score files.
    
    Args:
        input_folder (str): Folder containing mission score files
        output_folder (str): Folder to save the CSV file (default: same as input_folder)
        output_csv_name (str): Name of the output CSV file (default: "mission_scores")
        ignored_regions (int or None): Filter by number of ignored regions (default: None, include all)
    
    Returns:
        Path: Path to the generated CSV file
    """
    input_path = Path(input_folder)
    # output_path = Path(output_folder) if output_folder else input_path.parent
    output_path = Path(output_folder) if output_folder else input_path
    
    if input_folder[-1] == "/":
        input_folder = input_folder[:-1]
    if output_folder and output_folder[-1] == "/":
        output_folder = output_folder[:-1]
    
    id = input_folder.split("/")[-1]
    # print("ID: ", id)
    
    
    # if ends with .csv, remove it
    if output_csv_name.endswith(".csv"):
        output_csv_name = output_csv_name[:-4]
        
    output_csv = output_path / (output_csv_name + "_" + id + ".csv") \
                 if (output_folder is not None) else input_path / (output_csv_name + ".csv")
                 
    # print(f"output_csv: {output_csv}")

    files = sorted(input_path.glob("mission_score*.*txt"))  # Score files are .txt
    headers = [
        'MissionID', 'Algorithm', 'DroneCount', 'Deadline', 'IgnoredRegions',
        'TotalScore', 'Completeness', 'TimeEfficiency', 'CoverageScore',
        'RedundantPenalty', 'LatestDetTime', 'AvgDetTime', 'MedDetTime',
        'FiresDetected', 'TotalFires', 'TotalDetections', 'AreaCoverage'
    ]

    # Counter to track the number of missions for each algorithm
    algorithm_counter = Counter()
    # Dictionary to track the drones involved in each algorithm's missions
    algorithm_drone_counts = defaultdict(list)
    # Track filtered vs total missions
    total_files = len(files)
    filtered_files = 0

    with open(output_csv, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=headers)
        writer.writeheader()
        mission_id = 1  # Start with mission ID 1
        
        for file in files:
            data = parse_score_file(file)
            
            # Skip if ignored_regions filter is set and doesn't match
            if ignored_regions is not None and data['IgnoredRegions'] != ignored_regions:
                continue
                
            filtered_files += 1
            
            row = {
                'MissionID': mission_id,
                'Algorithm': data['Algorithm'],
                'DroneCount': data['DroneCount'],
                'Deadline': data['Deadline'],
                'IgnoredRegions': data['IgnoredRegions'],
                'TotalScore': data['TotalScore'],
                'Completeness': data['Completeness'],
                'TimeEfficiency': data['TimeEfficiency'],
                'CoverageScore': data['CoverageScore'],
                'RedundantPenalty': data['RedundantPenalty'],
                'LatestDetTime': data['LatestDetTime'],
                'AvgDetTime': data['AvgDetTime'],
                'MedDetTime': data['MedDetTime'],
                'FiresDetected': data['FiresDetected'],
                'TotalFires': data['TotalFires'],
                'TotalDetections': data['TotalDetections'],
                'AreaCoverage': data['AreaCoverage']
            }
            writer.writerow(row)
            
            # Count missions by algorithm
            if data['Algorithm']:
                algorithm_counter[data['Algorithm']] += 1
                # Track drone counts for each algorithm
                if data['DroneCount']:
                    algorithm_drone_counts[data['Algorithm']].append(data['DroneCount'])
                    
            mission_id += 1  # Increment mission ID for each included mission

    # Print the count of missions by algorithm and drone counts
    print("\nMission count by algorithm:")
    if algorithm_counter:
        for algorithm, count in algorithm_counter.items():
            print(f"  {algorithm}: {count} missions")
            
            # Print drone count information for this algorithm
            drone_counts = algorithm_drone_counts[algorithm]
            if drone_counts:
                drone_counter = Counter(drone_counts)
                print(f"    Drone distribution:")
                for drone_count, frequency in sorted(drone_counter.items()):
                    print(f"      {drone_count} drones: {frequency} missions")
               
    else:
        print("  No algorithm data found in the mission files")

    # Print filter information
    if ignored_regions is not None:
        print(f"\nFilter: Ignored Regions = {ignored_regions}")
        print(f"Missions included: {filtered_files} out of {total_files} total missions")
    else:
        print(f"\nTotal missions processed: {filtered_files}")
        
    return output_csv


if __name__ == '__main__':
    # homefolder = os.path.expanduser("~")

    # simfolder = homefolder+"/moos-ivp-uav-base/missions/UAV_fly/mission_score/sim"

    parser = argparse.ArgumentParser(description='Generate mission score CSV from score files.')
    parser.add_argument('--input_folder', '--if', type=str, default="sim", help='Input folder containing score files.')
    parser.add_argument('--output_folder', '--of',type=str, default=None, help='Output folder for the CSV file.')
    parser.add_argument('--output_csv_name', '-n', type=str, default="mission_scores", help='Output CSV file name.')
    parser.add_argument('--ignored_regions', '--ir', type=int, default=None, help='Filter by number of ignored regions.')
    args = parser.parse_args()
    # Call the function with command line arguments
    generate_mission_score_csv(args.input_folder, args.output_folder, args.output_csv_name, args.ignored_regions)

