import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import argparse
import os
import sys
import numpy as np

from pathlib import Path
from generate_mission_csv import generate_mission_score_csv as generate_csv


def plot_score_distributions(df, output_dir, save):

    metrics = {
        "Completeness": ("Completeness", 50.0),
        "Coverage": ("CoverageScore", 10.0),
        "TimeEfficiency": ("TimeEfficiency", 50.0),
        "TotalScore": ("TotalScore", 100.0),
    }

    bins = np.arange(0, 110, 10)
    labels = [f"{i}-{i+10}" for i in bins[:-1]]
    selected_drones = [3, 6, 9]

    df = df[df["DroneCount"].isin(selected_drones)]

    # Get unique algorithms for overall palette creation
    unique_algorithms = df['Algorithm'].unique()
    # Create a dictionary mapping algorithm to a consistent color
    algo_color_map = {algo: sns.color_palette("Set2")[i % 8] for i, algo in enumerate(unique_algorithms)}

    total_simulations = len(df)
    for metric_label, (column, divisor) in metrics.items():
        fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(14, 10), sharey=True)
        axes = axes.flatten()

        # Initialize legend handles and labels
        legend_handles = []
        legend_labels = []

        for i, drone_count in enumerate(selected_drones):
            ax = axes[i]
            subset = df[df["DroneCount"] == drone_count].copy()
            subset["Bucket"] = pd.cut(
                100 * subset[column] / divisor, bins=bins, labels=labels, right=True
            )

            bucketed = subset.groupby(["Algorithm", "Bucket"], observed=False).size().reset_index(name="Count")

            # Use the global color mapping for consistency across all drone counts
            sns.barplot(
                data=bucketed,
                x="Bucket",
                y="Count",
                hue="Algorithm",
                palette=algo_color_map,
                ax=ax
            )

            ax.set_title(f"{metric_label} (Drones: {drone_count})")
            ax.set_xlabel("Score Bucket (%)")
            ax.set_ylabel("Missions")
            ax.set_xticks(range(len(labels)))
            ax.set_xticklabels(labels, rotation=45)

            # Remove individual legends
            if ax.get_legend() is not None:
                if i == 0:  # Capture legend handles from first plot
                    legend_handles = ax.get_legend().legend_handles
                    legend_labels = [text.get_text() for text in ax.get_legend().texts]
                ax.get_legend().remove()

        # Use the fourth subplot for a shared legend
        axes[3].axis('off')
        if legend_handles and len(legend_labels) > 0:
            axes[3].legend(
                handles=legend_handles, 
                labels=legend_labels, 
                title="Algorithm", 
                loc="center",
                fontsize=12
            )

        plt.suptitle(f"{metric_label} Distribution (Total Simulations: {total_simulations})", fontsize=16)
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        if save:
            os.makedirs(output_dir, exist_ok=True)
            base_name = f"{metric_label.lower()}_distribution"
            plt.savefig(os.path.join(output_dir, f"{base_name}.png"))
            plt.savefig(os.path.join(output_dir, f"{base_name}.eps"), format="eps")
        plt.show()


def plot_time_statistics(df, output_dir, save):
    """
    Plot average, median, and latest times across drone counts and algorithms.
    Shows statistics in a box plot-like fashion using lines in a 2x2 grid.
    """
    # Use the specific time columns from the data
    time_columns = {
        'Average': 'AvgDetTime',
        'Median': 'MedDetTime',
        'Latest': 'LatestDetTime'
    }
    
    selected_drones = [3, 6, 9]
    df_filtered = df[df["DroneCount"].isin(selected_drones)].copy()
    
    # Get the deadline value (assuming it's consistent across all missions)
    deadline = df_filtered['Deadline'].iloc[0]
    
    # Create a figure with 2x2 grid of subplots (one will be empty or for legend)
    fig, axes = plt.subplots(2, 2, figsize=(14, 10), sharex=True, sharey=True)
    axes = axes.flatten()
    statistics = list(time_columns.keys())
    
    # Get unique algorithms and create a consistent color mapping
    unique_algorithms = df_filtered['Algorithm'].unique()
    algo_color_map = {algo: sns.color_palette("Set2")[i % 8] for i, algo in enumerate(unique_algorithms)}
    
    # Create a legend-only subplot
    legend_handles = []
    legend_labels = []

    total_simulations = len(df_filtered)
    for i, (stat_name, column_name) in enumerate(time_columns.items()):
        ax = axes[i]
        
        # Create a pivot table for this statistic
        pivot_data = df_filtered.pivot_table(
            index='DroneCount', 
            columns='Algorithm',
            values=column_name,
            aggfunc='mean'  # Average across missions with same drone count and algorithm
        ).reset_index()
        
        # Plot lines for each algorithm
        for j, algorithm in enumerate(pivot_data.columns[1:]):  # Skip DroneCount column
            line = ax.plot(
                pivot_data['DroneCount'], 
                pivot_data[algorithm],
                marker=["o", "s", "D", "^", "*"][j % 5],
                linestyle=["-", "--", "-.", ":", "-"][j % 5],
                linewidth=2,
                markersize=8,
                label=algorithm,
                color=algo_color_map[algorithm]  # Use consistent color mapping
            )[0]
            
            # Create handles for the legend
            if i == 0:
                legend_handles.append(line)
                legend_labels.append(algorithm)
        
        # Add a horizontal line for the deadline
        deadline_line = ax.axhline(y=deadline, color='red', linestyle='-', linewidth=2, alpha=0.7)
        
        # Add deadline to legend only once
        if i == 0:
            legend_handles.append(deadline_line)
            legend_labels.append(f"Deadline ({deadline}s)")
        
        ax.set_title(f"{stat_name} Time by Drone Count")
        ax.set_xlabel("Number of Drones")
        ax.set_ylabel("Time (seconds)")
        
        # Set x-axis ticks explicitly to selected drone counts
        ax.set_xticks(selected_drones)
        
        # Remove individual legends
        if ax.get_legend() is not None:
            ax.get_legend().remove()
    
    # Use the fourth subplot for a shared legend
    axes[3].axis('off')
    
    # Use explicit handles and labels for the legend
    if legend_handles and len(legend_labels) > 0:
        fig.legend(
            handles=legend_handles, 
            labels=legend_labels, 
            title="Algorithm", 
            loc="center",
            bbox_to_anchor=(0.5, 0.5),
            bbox_transform=axes[3].transAxes
        )

    plt.suptitle(f"Time Statistics (Total Simulations: {total_simulations})", fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    if save:
        os.makedirs(output_dir, exist_ok=True)
        plt.savefig(os.path.join(output_dir, "time_statistics.png"))
        plt.savefig(os.path.join(output_dir, "time_statistics.eps"), format="eps")
    plt.show()


def plot_avg_metrics_by_drone_count(df, output_dir, save):
    """
    Plot average TotalScore and AreaCoverage per drone count for each algorithm.
    Creates one consolidated plot with two metrics side by side.
    """
    # Define the metrics we want to plot
    metrics = {
        'Total Score': 'TotalScore',
        'Area Coverage (%)': 'AreaCoverage'
    }
    
    selected_drones = [3, 6, 9]
    df_filtered = df[df["DroneCount"].isin(selected_drones)].copy()
    
    # Create a figure with subplots for each metric
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), sharey=False)
    
    # Get unique algorithms and create a consistent color mapping
    unique_algorithms = df_filtered['Algorithm'].unique()
    algo_color_map = {algo: sns.color_palette("Set2")[i % 8] for i, algo in enumerate(unique_algorithms)}
    
    total_simulations = len(df_filtered)
    for i, (metric_name, column_name) in enumerate(metrics.items()):
        ax = axes[i]
        
        # Group by algorithm and drone count to calculate the average of the metric
        avg_metric = df_filtered.groupby(['Algorithm', 'DroneCount'])[column_name].mean().reset_index()
        
        # Plot lines for each algorithm
        for j, algorithm in enumerate(unique_algorithms):
            algo_data = avg_metric[avg_metric['Algorithm'] == algorithm]
            
            # Sort by drone count for proper line plotting
            algo_data = algo_data.sort_values('DroneCount')
            
            ax.plot(
                algo_data['DroneCount'],
                algo_data[column_name],
                marker='o',
                linestyle='-',
                linewidth=2,
                markersize=8,
                label=algorithm,
                color=algo_color_map[algorithm]
            )
        
        ax.set_title(f"Average {metric_name} by Drone Count")
        ax.set_xlabel("Number of Drones")
        ax.set_ylabel(metric_name)
        
        # Set x-axis ticks explicitly to selected drone counts
        ax.set_xticks(selected_drones)
        
        # Add grid for better readability
        ax.grid(True, linestyle='--', alpha=0.7)
        
        # Only add legend to the first plot
        if i == 0:
            ax.legend(title="Algorithm")


    plt.suptitle(f"Average metric by Drone Count (Total Simulations: {total_simulations})", fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    if save:
        os.makedirs(output_dir, exist_ok=True)
        plt.savefig(os.path.join(output_dir, "avg_metrics_by_drone_count.png"))
        plt.savefig(os.path.join(output_dir, "avg_metrics_by_drone_count.eps"), format="eps")
    plt.show()


def plot_fires_detected_by_drone_count(df, output_dir, save):
    """
    Create a scatter plot showing the number of detected fires by algorithm and drone count.
    Y-axis: Drone count (3, 6, 9)
    X-axis: Number of fires detected
    """
    selected_drones = [3, 6, 9]
    df_filtered = df[df["DroneCount"].isin(selected_drones)].copy()
    
    plt.figure(figsize=(12, 8))
    
    # Get unique algorithms and create a consistent color mapping
    unique_algorithms = df_filtered['Algorithm'].unique()
    algo_color_map = {algo: sns.color_palette("Set2")[i % 8] for i, algo in enumerate(unique_algorithms)}
    
    # Create a dictionary to map drone counts to y-positions
    drone_positions = {count: i for i, count in enumerate(selected_drones)}
    
    # Add small random jitter to y-positions for better visualization
    jitter = 0.1
    
    total_simulations = len(df_filtered)
    # Plot each data point
    for idx, row in df_filtered.iterrows():
        # Add small random vertical jitter for better visualization
        y_pos = drone_positions[row['DroneCount']] + np.random.uniform(-jitter, jitter)
        
        # Plot the point
        plt.scatter(
            row['FiresDetected'], 
            y_pos,
            s=100,  # Point size
            color=algo_color_map[row['Algorithm']],
            alpha=0.7,
            edgecolors='black',
            linewidths=0.5,
            zorder=3  # Ensure points are on top of other elements
        )
    
    # Add total fires as a reference point (using a different marker)
    for idx, row in df_filtered.iterrows():
        y_pos = drone_positions[row['DroneCount']]
        
        # Use 'X' to mark total fires
        plt.scatter(
            row['TotalFires'], 
            y_pos,
            s=60,  # Point size
            marker='X',
            color='gray',
            alpha=0.5,
            zorder=2  # Behind colored points but in front of grid
        )
    
    # Create a legend for algorithms
    legend_elements = [plt.Line2D([0], [0], marker='o', color='w', 
                                 markerfacecolor=algo_color_map[algo], markersize=10, label=algo)
                     for algo in unique_algorithms]
    
    # Add a legend element for total fires
    legend_elements.append(plt.Line2D([0], [0], marker='X', color='w', 
                                     markerfacecolor='gray', markersize=8, 
                                     label='Total Fires Available'))
    
    plt.legend(handles=legend_elements, loc='upper left', title='Algorithm')
    
    # Set custom y-ticks at the drone count positions
    plt.yticks(list(drone_positions.values()), list(drone_positions.keys()))
    
    # Add horizontal grid lines
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    
    # Set labels and title
    plt.xlabel('Number of Fires Detected', fontsize=12)
    plt.ylabel('Number of Drones', fontsize=12)
    plt.title(f"Fires Detected by Drone Count and Algorithm (Total Simulations: {total_simulations})", fontsize=14)

    # Add a text annotation explaining the X markers
    plt.figtext(0.5, 0.01, 'Gray X markers indicate the total number of fires available in the mission', 
               ha='center', fontsize=10, style='italic')
    
    plt.tight_layout()
    if save:
        os.makedirs(output_dir, exist_ok=True)
        plt.savefig(os.path.join(output_dir, "fires_detected_scatter.png"))
        plt.savefig(os.path.join(output_dir, "fires_detected_scatter.eps"), format="eps")
    plt.show()


def plot_mission_scores(
    csv_name="mission_scores",
    output_dir="plots",
    save=True,
    regenerate=False,
    score_dir="sim"
):
    csv_path = score_dir + "/" + csv_name
    output_dir = output_dir
    
    if regenerate:
        print(f"📄 Regenerating CSV from score files in: {score_dir}")
        generate_csv(input_folder=score_dir, output_csv_name=csv_name)

    if not os.path.exists(csv_path):
        print(f"❌ CSV file not found at {csv_path}")
        sys.exit(1)

    df = pd.read_csv(csv_path)

    ######### Fill all the missing Algorithm values with "Unknown" #########
    df["Algorithm"] = df["Algorithm"].fillna("UNKNOWN") 
    
    # remove rows with NaN values in the "Algorithm" column
    # df = df.dropna(subset=["Algorithm"])

    # Check for required columns
    required_columns = [
        "MissionID", "TotalScore", "Algorithm", "TimeEfficiency", 
        "Completeness", "CoverageScore", "DroneCount", "AreaCoverage", 
        "TotalFires", "FiresDetected"
    ]
    missing_columns = [col for col in required_columns if col not in df.columns]
    if missing_columns:
        print(f"❌ Missing columns in CSV: {missing_columns}")
        sys.exit(1)

    
    
    
    sns.set_theme(style="whitegrid")
    plt.rcParams["figure.autolayout"] = True
    os.makedirs(output_dir, exist_ok=True)

    def save_plot(name):
        if save:
            plt.savefig(os.path.join(output_dir, f"{name}.png"))
            plt.savefig(os.path.join(output_dir, f"{name}.eps"), format='eps')


    ####### PLOTTING #######
    

    # Print the df
    # print(f"df\n {df}")
        
    
    plot_score_distributions(df, output_dir, save)
    
    # Plot time statistics
    plot_time_statistics(df, output_dir, save)
    
    # Plot average metrics by drone count
    plot_avg_metrics_by_drone_count(df, output_dir, save)
    
    # Plot fires detected scatter
    plot_fires_detected_by_drone_count(df, output_dir, save)

    # # A. Total Score per Mission
    # plt.figure(figsize=(10, 5))
    # sns.barplot(data=df, x="MissionID", y="TotalScore", hue="Algorithm")
    # plt.title("Total Score per Mission")
    # save_plot("total_score_per_mission")
    # plt.show()

    # # B. Box plots for Score Components
    # for metric in ["TimeEfficiency", "Completeness", "CoverageScore"]:
    #     plt.figure(figsize=(8, 5))
    #     sns.boxplot(data=df, x="Algorithm", y=metric)
    #     plt.title(f"{metric} by Algorithm")
    #     save_plot(f"boxplot_{metric.lower().replace(' ', '_')}")
    #     plt.show()

    # # C. Scatter plot: Total Score vs. Drone Count
    # plt.figure(figsize=(8, 5))
    # sns.scatterplot(data=df, x="DroneCount", y="TotalScore", hue="Algorithm", s=100)
    # plt.title("Total Score vs. Drone Count")
    # save_plot("total_score_vs_drone_count")
    # plt.show()

    # # D. Line plot: Area Coverage over MissionID
    # plt.figure(figsize=(10, 5))
    # sns.lineplot(data=df, x="MissionID", y="AreaCoverage", hue="Algorithm", marker="o")
    # plt.title("Area Coverage across Missions")
    # save_plot("area_coverage_per_mission")
    # plt.show()

    # # E. Stacked bar: Fires Detected vs Missed
    # plt.figure(figsize=(10, 5))
    # df["FiresMissed"] = df["TotalFires"] - df["FiresDetected"]
    # fires_df = df[["MissionID", "FiresDetected", "FiresMissed"]].set_index("MissionID")
    # fires_df.sort_index(inplace=True)
    # fires_df.plot(kind="bar", stacked=True, colormap="Set1")
    # plt.title("Fires Detected vs Missed")
    # plt.ylabel("Count")
    # plt.xticks(rotation=45)
    # plt.tight_layout()
    # save_plot("fires_detected_vs_missed")
    # plt.show()

    print(f"✅ Plotting complete. Saved to: {output_dir}" if save else "✅ Plotting complete. No files saved.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot mission score results.")
    parser.add_argument("--csv", default="mission_scores.csv", help="name of the  CSV file.")
    parser.add_argument("--out", default="plots", help="Directory to save plots.")
    parser.add_argument("--save", action="store_true", help="Save plots to file.")
    parser.add_argument("--regenerate", action="store_true", help="Regenerate CSV file before plotting.")
    parser.add_argument("--score_dir", default="sim", help="Directory containing mission score files.")
    args = parser.parse_args()

    plot_mission_scores(
        csv_name=args.csv,
        output_dir=args.out,
        save=args.save,
        regenerate=args.regenerate,
        score_dir=args.score_dir,
    )
