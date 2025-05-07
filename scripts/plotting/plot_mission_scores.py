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
    import numpy as np

    metrics = {
        "Completeness": ("Completeness", 50.0),
        "Coverage": ("CoverageScore", 10.0),
        "TimeEfficiency": ("TimeEfficiency", 50.0),
        "TotalScore": ("TotalScore", 100.0),
    }

    bins = np.arange(0, 110, 10)
    labels = [f"{i}-{i+10}" for i in bins[:-1]]
    palette = sns.color_palette("Set2")
    selected_drones = [1, 3, 6, 9]

    df = df[df["DroneCount"].isin(selected_drones)]

    for metric_label, (column, divisor) in metrics.items():
        fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(14, 10), sharey=True)
        axes = axes.flatten()

        for i, drone_count in enumerate(selected_drones):
            ax = axes[i]
            subset = df[df["DroneCount"] == drone_count].copy()
            subset["Bucket"] = pd.cut(
                100 * subset[column] / divisor, bins=bins, labels=labels, right=False
            )

            bucketed = subset.groupby(["Algorithm", "Bucket"], observed=False).size().reset_index(name="Count")

            sns.barplot(
                data=bucketed,
                x="Bucket",
                y="Count",
                hue="Algorithm",
                palette=palette,
                ax=ax
            )
            ax.set_title(f"{metric_label} (Drones: {drone_count})")
            ax.set_xlabel("Score Bucket (%)")
            ax.set_ylabel("Missions")
            ax.set_xticks(range(len(labels)))
            ax.set_xticklabels(labels, rotation=45)
            if i == 0:
                ax.legend(title="Algorithm")
            else:
                legend = ax.get_legend()
                if legend is not None:
                    legend.remove()


        # Remove unused axes if any
        for j in range(len(selected_drones), 4):
            fig.delaxes(axes[j])

        plt.tight_layout()
        if save:
            os.makedirs(output_dir, exist_ok=True)
            base_name = f"{metric_label.lower()}_distribution"
            plt.savefig(os.path.join(output_dir, f"{base_name}.png"))
            plt.savefig(os.path.join(output_dir, f"{base_name}.eps"), format="eps")
        plt.show()





def plot_mission_scores(
    csv_name="mission_scores.csv",
    output_dir="plots",
    save=True,
    regenerate=False,
    score_dir="sim"
):
    csv_path = score_dir + "/" + csv_name
    output_dir = score_dir + "/" + output_dir
    
    if regenerate:
        print(f"📄 Regenerating CSV from score files in: {score_dir}")
        generate_csv(score_dir, None, csv_name)

    if not os.path.exists(csv_path):
        print(f"❌ CSV file not found at {csv_path}")
        sys.exit(1)

    df = pd.read_csv(csv_path)

    ######### Fill all the missing Algorithm values with "Unknown" #########
    df["Algorithm"] = df["Algorithm"].fillna("Unknown") 
    
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
    


    plot_score_distributions(df, output_dir, save)

    # A. Total Score per Mission
    plt.figure(figsize=(10, 5))
    sns.barplot(data=df, x="MissionID", y="TotalScore", hue="Algorithm")
    plt.title("Total Score per Mission")
    save_plot("total_score_per_mission")
    plt.show()

    # B. Box plots for Score Components
    for metric in ["TimeEfficiency", "Completeness", "CoverageScore"]:
        plt.figure(figsize=(8, 5))
        sns.boxplot(data=df, x="Algorithm", y=metric)
        plt.title(f"{metric} by Algorithm")
        save_plot(f"boxplot_{metric.lower().replace(' ', '_')}")
        plt.show()

    # C. Scatter plot: Total Score vs. Drone Count
    plt.figure(figsize=(8, 5))
    sns.scatterplot(data=df, x="DroneCount", y="TotalScore", hue="Algorithm", s=100)
    plt.title("Total Score vs. Drone Count")
    save_plot("total_score_vs_drone_count")
    plt.show()

    # D. Line plot: Area Coverage over MissionID
    plt.figure(figsize=(10, 5))
    sns.lineplot(data=df, x="MissionID", y="AreaCoverage", hue="Algorithm", marker="o")
    plt.title("Area Coverage across Missions")
    save_plot("area_coverage_per_mission")
    plt.show()

    # E. Stacked bar: Fires Detected vs Missed
    plt.figure(figsize=(10, 5))
    df["FiresMissed"] = df["TotalFires"] - df["FiresDetected"]
    fires_df = df[["MissionID", "FiresDetected", "FiresMissed"]].set_index("MissionID")
    fires_df.sort_index(inplace=True)
    fires_df.plot(kind="bar", stacked=True, colormap="Set1")
    plt.title("Fires Detected vs Missed")
    plt.ylabel("Count")
    plt.xticks(rotation=45)
    plt.tight_layout()
    save_plot("fires_detected_vs_missed")
    plt.show()

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
