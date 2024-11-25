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

def plot_combined_motor_data(df_with_filter, df_without_filter):
    """
    Plots the four parameters with both 'with filter' and 'without filter' data on the same graphs.

    Args:
        df_with_filter (pd.DataFrame): DataFrame with spike filter active.
        df_without_filter (pd.DataFrame): DataFrame with spike filter inactive.
    """
    # Define plot configurations for each parameter
    plot_params = [
        {
            'title': 'Potentiometer Raw Values',
            'y_label': 'Raw ADC Value',
            'with_data': df_with_filter['raw'],
            'without_data': df_without_filter['raw'],
            'with_color': 'blue',
            'without_color': 'orange'
        },
        {
            'title': 'Current Angle (Degrees)',
            'y_label': 'Angle (°)',
            'with_data': df_with_filter['angle'],
            'without_data': df_without_filter['angle'],
            'with_color': 'green',
            'without_color': 'red'
        },
        {
            'title': 'Error to Target (Degrees)',
            'y_label': 'Error (°)',
            'with_data': df_with_filter['error'],
            'without_data': df_without_filter['error'],
            'with_color': 'purple',
            'without_color': 'brown'
        },
        {
            'title': 'Control Signal Percentage',
            'y_label': 'Control Signal (%)',
            'with_data': df_with_filter['control_signal'],
            'without_data': df_without_filter['control_signal'],
            'with_color': 'cyan',
            'without_color': 'magenta'
        }
    ]

    # Create a figure with 2x2 subplots
    fig, axs = plt.subplots(2, 2, figsize=(18, 12))
    fig.suptitle('Motor 3 Control Parameters: With vs Without Spike Filter', fontsize=20)

    # Flatten the axs array for easy iteration
    axs = axs.flatten()

    for idx, param in enumerate(plot_params):
        ax = axs[idx]
        ax.plot(df_with_filter['relative_time'], param['with_data'],
                label='With Spike Filter', color=param['with_color'])
        ax.plot(df_without_filter['relative_time'], param['without_data'],
                label='Without Spike Filter', color=param['without_color'], linestyle='--')
        ax.set_title(param['title'], fontsize=16)
        ax.set_xlabel('Time (s)', fontsize=14)
        ax.set_ylabel(param['y_label'], fontsize=14)
        ax.legend(fontsize=12)
        ax.grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig('motor3_with_and_without_spike_filter_plot.png')
    plt.show()

def main():
    # Define log file paths
    log_with_filter = 'motor3_with_spike_filter.log'
    log_without_filter = 'motor3_without_spike_filter.log'

    # Parse log files
    print("Parsing log files...")
    df_with_filter = parse_log_file(log_with_filter)
    df_without_filter = parse_log_file(log_without_filter)

    # Check if data was found
    if df_with_filter.empty:
        print(f"No data to plot from {log_with_filter}. Exiting.")
        return
    if df_without_filter.empty:
        print(f"No data to plot from {log_without_filter}. Exiting.")
        return

    # Plot the data
    print("Plotting data...")
    plot_combined_motor_data(df_with_filter, df_without_filter)
    print("Plot saved as 'motor3_with_and_without_spike_filter_plot.png' and displayed.")

if __name__ == "__main__":
    main()
