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
    Plots all four parameters on a single graph with multiple y-axes.

    Args:
        df (pd.DataFrame): DataFrame containing the data to plot.
        title (str): Title of the figure.
        output_filename (str): Filename to save the plot.
    """
    fig, ax1 = plt.subplots(figsize=(15, 8))
    fig.suptitle(title, fontsize=20)

    # Plot Potentiometer Raw Values on ax1
    color1 = 'tab:blue'
    ax1.set_xlabel('Time (s)', fontsize=14)
    ax1.set_ylabel('Raw ADC Value', color=color1, fontsize=14)
    ax1.plot(df['relative_time'], df['raw'], color=color1, label='Raw ADC Value')
    ax1.tick_params(axis='y', labelcolor=color1)

    # Create a second y-axis for Angle
    ax2 = ax1.twinx()
    color2 = 'tab:green'
    ax2.set_ylabel('Angle (°)', color=color2, fontsize=14)
    ax2.plot(df['relative_time'], df['angle'], color=color2, label='Angle (°)')
    ax2.tick_params(axis='y', labelcolor=color2)

    # Create a third y-axis for Error
    ax3 = ax1.twinx()
    color3 = 'tab:purple'
    ax3.set_ylabel('Error (°)', color=color3, fontsize=14)
    ax3.plot(df['relative_time'], df['error'], color=color3, label='Error (°)')
    ax3.tick_params(axis='y', labelcolor=color3)

    # Offset the third y-axis
    ax3.spines['right'].set_position(('outward', 60))

    # Create a fourth y-axis for Control Signal
    ax4 = ax1.twinx()
    color4 = 'tab:cyan'
    ax4.set_ylabel('Control Signal (%)', color=color4, fontsize=14)
    ax4.plot(df['relative_time'], df['control_signal'], color=color4, label='Control Signal (%)')
    ax4.tick_params(axis='y', labelcolor=color4)

    # Offset the fourth y-axis
    ax4.spines['right'].set_position(('outward', 120))

    # Add legends
    lines_labels = [ax.get_legend_handles_labels() for ax in [ax1, ax2, ax3, ax4]]
    lines, labels = [sum(lol, []) for lol in zip(*lines_labels)]
    ax1.legend(lines, labels, loc='upper left', fontsize=12)

    # Add grid
    ax1.grid(True)

    # Adjust layout to prevent overlap
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    # Save and show the plot
    plt.savefig(output_filename)
    plt.show()

def main():
    # Define log file paths
    log_with_filter = 'motor3_with_spike_filter.log'
    log_without_filter = 'motor3_without_spike_filter.log'

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
