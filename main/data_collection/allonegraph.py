import re
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
import os

def parse_log_file(file_path):
    """
    Parses the given log file and extracts timestamp, raw ADC value,
    angle, error, and control signal percentage.

    Args:
        file_path (str): Path to the log file.

    Returns:
        pd.DataFrame: DataFrame containing the extracted data with relative time.
    """
    # Regular expression pattern to match the [INFO] M3 | Raw: ... lines
    pattern = re.compile(
        r'(?P<timestamp>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}) '
        r'\[INFO\] M3 \| Raw: (?P<raw>\d+) \| Angle: (?P<angle>[\d.]+)° \| '
        r'Error: (?P<error>[-\d.]+)° \| Control Signal: (?P<control_signal>[\d.]+)%'
    )

    data = []

    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
        return pd.DataFrame()  # Return empty DataFrame

    with open(file_path, 'r') as f:
        for line in f:
            match = pattern.search(line)
            if match:
                # Parse timestamp
                timestamp_str = match.group('timestamp')
                timestamp = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S,%f')
                
                # Extract parameters
                raw = int(match.group('raw'))
                angle = float(match.group('angle'))
                error = float(match.group('error'))
                control_signal = float(match.group('control_signal'))

                data.append({
                    'timestamp': timestamp,
                    'raw': raw,
                    'angle': angle,
                    'error': error,
                    'control_signal': control_signal
                })

    if not data:
        print(f"No matching data found in {file_path}.")
        return pd.DataFrame()  # Return empty DataFrame

    # Create DataFrame
    df = pd.DataFrame(data)

    # Sort by timestamp in case it's unordered
    df = df.sort_values('timestamp').reset_index(drop=True)

    # Calculate relative time in seconds
    df['relative_time'] = (df['timestamp'] - df['timestamp'].iloc[0]).dt.total_seconds()

    return df

def plot_combined_graph(df, title, output_filename):
    """
    Plots all four parameters on a single graph with markers for data points.

    Args:
        df (pd.DataFrame): DataFrame containing the data to plot.
        title (str): Title of the figure.
        output_filename (str): Filename to save the plot.
    """
    fig, ax = plt.subplots(figsize=(15, 8))
    fig.suptitle(title, fontsize=20)

    # Plot all data with markers
    ax.plot(df['relative_time'], df['raw'], label='Raw ADC Value', marker='o', markersize=4, linestyle='-', linewidth=1)
    ax.plot(df['relative_time'], df['angle'], label='Angle (°)', marker='s', markersize=4, linestyle='-', linewidth=1)
    ax.plot(df['relative_time'], df['error'], label='Error (°)', marker='^', markersize=4, linestyle='-', linewidth=1)
    ax.plot(df['relative_time'], df['control_signal'], label='Control Signal (%)', marker='d', markersize=4, linestyle='-', linewidth=1)

    # Set labels and grid
    ax.set_xlabel('Time (s)', fontsize=14)
    ax.set_ylabel('Values', fontsize=14)
    ax.grid(True)

    # Add legend with markers
    lines, labels = ax.get_legend_handles_labels()
    ax.legend(lines, labels, loc='upper left', fontsize=12)

    # Adjust layout to prevent overlap
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    # Save and show the plot
    plt.savefig(output_filename)
    plt.show()

def main():
    # Define log file paths
    log_with_filter = 'motor3_with_spike_filter_5.log'
    log_without_filter = 'motor3_without_spike_filter_5.log'

    # Parse log files
    print("Parsing 'with spike filter' log file...")
    df_with_filter = parse_log_file(log_with_filter)
    print("Parsing 'without spike filter' log file...")
    df_without_filter = parse_log_file(log_without_filter)

    # Check if data was found
    if df_with_filter.empty:
        print(f"No data to plot from {log_with_filter}. Exiting.")
    else:
        # Plot data with spike filter
        print("Plotting data with spike filter...")
        plot_combined_graph(
            df_with_filter,
            "Motor 3 Control Parameters: With Spike Filter",
            "motor3_with_spike_filter_plot.png"
        )
        print("Plot saved as 'motor3_with_spike_filter_plot.png'.")

    if df_without_filter.empty:
        print(f"No data to plot from {log_without_filter}. Exiting.")
    else:
        # Plot data without spike filter
        print("Plotting data without spike filter...")
        plot_combined_graph(
            df_without_filter,
            "Motor 3 Control Parameters: Without Spike Filter",
            "motor3_without_spike_filter_plot.png"
        )
        print("Plot saved as 'motor3_without_spike_filter_plot.png'.")

    print("All plots generated successfully.")

if __name__ == "__main__":
    main()
