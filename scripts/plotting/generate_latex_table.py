import pandas as pd
import numpy as np
import argparse
import os

def generate_latex_table(csv_file, algorithms, drone_counts, save=False):
    """
    Generate a LaTeX table with statistics from a CSV file, combining specified algorithms and drone counts.
    
    Parameters:
    - csv_file (str): Path to the CSV file.
    - algorithms (list or str): List of algorithms or 'all' to include all algorithms.
    - drone_counts (list or str): List of drone counts or 'all' to include all drone counts.
    - save (bool): If True, save LaTeX output to a file (default: False).
    
    Returns:
    - tuple: (str, str or None) - LaTeX code and output filename (if save=True).
    """
    # Read CSV file
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        return f"Error: CSV file '{csv_file}' not found.", None
    
    # Get unique values
    unique_algorithms = df['Algorithm'].unique().tolist()
    unique_drone_counts = df['DroneCount'].unique().tolist()
    
    # Process algorithms
    if algorithms == 'all':
        algo_filter = unique_algorithms
        algo_label = "TMSTC* and Voronoi Search"
    else:
        algo_filter = algorithms.split(',')
        invalid_algos = [algo for algo in algo_filter if algo not in unique_algorithms]
        if invalid_algos:
            return f"Error: Invalid algorithms: {', '.join(invalid_algos)}.", None
        # Map algorithm names for display
        display_names = []
        for algo in algo_filter:
            if algo == "TMSTC_STAR":
                display_names.append("TMSTC*")
            else:
                display_names.append("Voronoi search")
        algo_label = ", ".join(display_names) if len(display_names) > 1 else display_names[0]
    
    # Process drone counts
    if drone_counts == 'all':
        drone_filter = unique_drone_counts
        drone_label = "All Drone Counts"
        # For label, use 'all' if drone_counts is 'all'
        drone_label_for_table = "all"
    else:
        try:
            drone_filter = [int(dc) for dc in drone_counts.split(',')]
            invalid_drones = [dc for dc in drone_filter if dc not in unique_drone_counts]
            if invalid_drones:
                return f"Error: Invalid drone counts: {', '.join(map(str, invalid_drones))}.", None
            drone_label = ", ".join(map(str, drone_filter)) if len(drone_filter) > 1 else str(drone_filter[0])
            # For label, convert drone count to words (e.g., 2 -> 'two')
            drone_count_words = {1: "one", 2: "two", 3: "three", 4: "four", 5: "five"}
            if len(drone_filter) == 1:
                drone_label_for_table = drone_count_words.get(drone_filter[0], str(drone_filter[0]))
            else:
                drone_label_for_table = "_".join([drone_count_words.get(dc, str(dc)) for dc in drone_filter])
        except ValueError:
            return "Error: --drone_count must be comma-separated integers or 'all'.", None
    
    # Filter data
    filtered_df = df[
        (df['Algorithm'].isin(algo_filter)) &
        (df['DroneCount'].isin(drone_filter))
    ]
    if filtered_df.empty:
        return f"Error: No data found for algorithms '{algo_label}' and drone counts '{drone_label}'.", None
    
    # Define metrics
    metrics = [
        'TotalScore', 'Completeness', 'TimeEfficiency', 'CoverageScore',
        'RedundantPenalty', 'LatestDetTime', 'AvgDetTime', 'MedDetTime', 'AreaCoverage'
    ]
    
    # Compute statistics
    stats = {}
    for metric in metrics:
        if metric in filtered_df.columns:
            data = filtered_df[metric].dropna()
            if not data.empty:
                avg = np.mean(data)
                std = np.std(data, ddof=1)  # Sample standard deviation
                low = np.min(data)
                high = np.max(data)
                stats[metric] = (avg, std, low, high)
            else:
                stats[metric] = (0.0, 0.0, 0.0, 0.0)
        else:
            stats[metric] = (0.0, 0.0, 0.0, 0.0)
    
    # LaTeX table template matching the provided format
    caption = f"Statistics of Metrics for {algo_label} Algorithm with {drone_label} Drones on the field"
    latex = r"""
\documentclass{article}
\usepackage[table]{xcolor} % For table coloring
\usepackage{pgf} % For calculations
\usepackage{colortbl} % For \cellcolor
\usepackage{booktabs} % For professional table formatting
\usepackage{siunitx} % For number formatting
\usepackage{hhline} % For hhline command

% Color definitions
\definecolor{highColor}{HTML}{76f013} % Green for highest value
\definecolor{lowColor}{HTML}{ec462e} % Red for lowest value
\newcommand*{\opacity}{90} % Opacity of background color

% Data set ranges
\newcommand*{\minval}{0.0} % Minimum for most metrics
\newcommand*{\minvalNeg}{-10} % Minimum for Redundant Penalty
\newcommand*{\maxvalHundred}{100.0} % Maximum for Total Score, Area Coverage
\newcommand*{\maxvalFifty}{50.0} % Maximum for Completeness, Time Efficiency
\newcommand*{\maxvalTen}{10.0} % Maximum for Coverage Score
\newcommand*{\maxvalSixhundred}{600.0} % Maximum for Detection Times
\newcommand*{\maxvalSixty}{60.0} % Maximum for Total Detections

% Gradient function for single cell
% \gradientcell{cell_val}{min_val}{max_val}{colorlow}{colorhigh}{opacity}
\newcommand{\gradientcell}[6]{%
    \pgfmathsetmacro{\cellval}{#1}% Store cell value
    \pgfmathsetmacro{\minval}{#2}% Store min value
    \pgfmathsetmacro{\maxval}{#3}% Store max value
    \pgfmathsetmacro{\denom}{\maxval - \minval}% Compute denominator
    \pgfmathsetmacro{\percentage}{and(\cellval >= \minval, \cellval <= \maxval) ? int(round(100 * (\cellval - \minval) / \denom)) : -1}% Compute gradient percentage or -1 if out of range
    \ifnum\percentage>-1\relax%
        \cellcolor{#5!\percentage!#4!#6}\num[minimum-decimal-digits=2]{#1}% Apply gradient color
    \else%
        \num[minimum-decimal-digits=2]{#1}% Output without coloring if out of range
    \fi%
}

% Metric-specific gradient commands
\newcommand{\gcH}[1]{% For Total Score, Area Coverage (0 to 100)
    \gradientcell{#1}{\minval}{\maxvalHundred}{\lowColor}{\highColor}{\opacity}%
}
\newcommand{\gcF}[1]{% For Completeness, Time Efficiency (0 to 50)
    \gradientcell{#1}{\minval}{\maxvalFifty}{\lowColor}{\highColor}{\opacity}%
}
\newcommand{\gcT}[1]{% For Coverage Score (0 to 10)
    \gradientcell{#1}{\minval}{\maxvalTen}{\lowColor}{\highColor}{\opacity}%
}
\newcommand{\gcN}[1]{% For Redundant Penalty (-10 to 0)
    \gradientcell{#1}{\minvalNeg}{\minval}{\lowColor}{\highColor}{\opacity}%
}
\newcommand{\gct}[1]{% For Detection Times (0 to 600)
    \gradientcell{#1}{\minval}{\maxvalSixhundred}{\lowColor}{\highColor}{\opacity}%
}
\newcommand{\gcTotalDetections}[1]{% For Total Detections (0 to 60)
    \gradientcell{#1}{\minval}{\maxvalSixty}{\highColor}{\lowColor}{\opacity}%
}

\begin{document}

\begin{table}[!h]
    \centering
    \caption{""" + caption + r"""}
    \label{tab:field:""" + drone_label_for_table + r""":fire_mission_scenarios_stats}
    \begin{tabular}{|l|c|c|c|c|}
        \hhline{~*{4}{-}}
        \multicolumn{1}{c|}{} & \multicolumn{4}{c|}{\textbf{""" + algo_label + r""" Algorithm - Field}} \\
        \hline
        \textbf{Metric} & \textbf{Average} & \textbf{Std. Dev.} & \textbf{Low} & \textbf{High} \\
        \hline
        Total Score & \gcH{""" + f"{stats['TotalScore'][0]:.2f}" + r"""} & """ + f"{stats['TotalScore'][1]:.3f}" + r""" & \gcH{""" + f"{stats['TotalScore'][2]:.2f}" + r"""} & \gcH{""" + f"{stats['TotalScore'][3]:.2f}" + r"""} \\
        Completeness [50 max] & \gcF{""" + f"{stats['Completeness'][0]:.2f}" + r"""} & """ + f"{stats['Completeness'][1]:.3f}" + r""" & \gcF{""" + f"{stats['Completeness'][2]:.2f}" + r"""} & \gcF{""" + f"{stats['Completeness'][3]:.2f}" + r"""} \\
        Time Efficiency [50 max] & \gcF{""" + f"{stats['TimeEfficiency'][0]:.2f}" + r"""} & """ + f"{stats['TimeEfficiency'][1]:.3f}" + r""" & \gcF{""" + f"{stats['TimeEfficiency'][2]:.2f}" + r"""} & \gcF{""" + f"{stats['TimeEfficiency'][3]:.2f}" + r"""} \\
        % Coverage Score [10 max] & \gcT{""" + f"{stats['CoverageScore'][0]:.2f}" + r"""} & """ + f"{stats['CoverageScore'][1]:.3f}" + r""" & \gcT{""" + f"{stats['CoverageScore'][2]:.2f}" + r"""} & \gcT{""" + f"{stats['CoverageScore'][3]:.2f}" + r"""} \\
        % Redundant Penalty [-10 min] & \gcN{""" + f"{stats['RedundantPenalty'][0]:.2f}" + r"""} & """ + f"{stats['RedundantPenalty'][1]:.3f}" + r""" & \gcN{""" + f"{stats['RedundantPenalty'][2]:.2f}" + r"""} & \gcN{""" + f"{stats['RedundantPenalty'][3]:.2f}" + r"""} \\
        Latest Detection Time [s] & \gct{""" + f"{stats['LatestDetTime'][0]:.2f}" + r"""} & """ + f"{stats['LatestDetTime'][1]:.3f}" + r""" & \gct{""" + f"{stats['LatestDetTime'][2]:.2f}" + r"""} & \gct{""" + f"{stats['LatestDetTime'][3]:.2f}" + r"""} \\
        Average Detection Time [s] & \gct{""" + f"{stats['AvgDetTime'][0]:.2f}" + r"""} & """ + f"{stats['AvgDetTime'][1]:.3f}" + r""" & \gct{""" + f"{stats['AvgDetTime'][2]:.2f}" + r"""} & \gct{""" + f"{stats['AvgDetTime'][3]:.2f}" + r"""} \\
        Median Detection Time [s] & \gct{""" + f"{stats['MedDetTime'][0]:.2f}" + r"""} & """ + f"{stats['MedDetTime'][1]:.3f}" + r""" & \gct{""" + f"{stats['MedDetTime'][2]:.2f}" + r"""} & \gct{""" + f"{stats['MedDetTime'][3]:.2f}" + r"""} \\
        % Total Detections & """ + f"{stats.get('TotalDetections', (0.0, 0.0, 0.0, 0.0))[0]:.2f}" + r""" & """ + f"{stats.get('TotalDetections', (0.0, 0.0, 0.0, 0.0))[1]:.3f}" + r""" & """ + f"{stats.get('TotalDetections', (0.0, 0.0, 0.0, 0.0))[2]:.2f}" + r""" & """ + f"{stats.get('TotalDetections', (0.0, 0.0, 0.0, 0.0))[3]:.2f}" + r""" \\
        Area Coverage [\%] & \gcH{""" + f"{stats['AreaCoverage'][0]:.2f}" + r"""} & """ + f"{stats['AreaCoverage'][1]:.3f}" + r""" & \gcH{""" + f"{stats['AreaCoverage'][2]:.2f}" + r"""} & \gcH{""" + f"{stats['AreaCoverage'][3]:.2f}" + r"""} \\
        \hline
    \end{tabular}
\end{table}

\end{document}
"""
    # Suggest output filename
    algo_part = "all" if algorithms == 'all' else "_".join(algo_filter).replace(" ", "_")
    drone_part = "all" if drone_counts == 'all' else "_".join(map(str, drone_filter))
    output_file = f"{algo_part}_drone{drone_part}_table.tex" if save else None
    
    return latex, output_file

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Generate a LaTeX table from mission scores CSV.")
    parser.add_argument(
        "--csv_file",
        type=str,
        default="mission_scores.csv",
        help="Path to the CSV file (default: mission_scores.csv)"
    )
    parser.add_argument(
        "--algorithm",
        type=str,
        default="TMSTC_STAR",
        help="Algorithm(s) to filter, comma-separated, or 'all' [TMSTC_STAR, VORONOI_SEARCH] (default: TMSTC_STAR)"
    )
    parser.add_argument(
        "--drone_count",
        type=str,
        default="2",
        help="Drone count(s) to filter, comma-separated, or 'all' (default: 2)"
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Save LaTeX output to a file (default: False)"
    )
    
    # Parse arguments
    args = parser.parse_args()
    
    # Generate LaTeX table
    latex_code, output_file = generate_latex_table(
        csv_file=args.csv_file,
        algorithms=args.algorithm,
        drone_counts=args.drone_count,
        save=args.save
    )
    
    # Print LaTeX code
    print(latex_code)
    
    # Save if requested
    if args.save and output_file and not output_file.startswith("Error:"):
        with open(output_file, "w") as f:
            f.write(latex_code)
        print(f"LaTeX table saved to {output_file}")

if __name__ == "__main__":
    main()