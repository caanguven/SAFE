import re
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt

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

def plot_motor_data(df_with_filter, df_without_filter):
    """
    Plots the four parameters from both DataFrames on separate subplots.

    Args:
        df_with_filter (pd.DataFrame): DataFrame with spike filter active.
        df_without_filter (pd.DataFrame): DataFrame with spike filter inactive.
    """
    # Create a 2x2 subplot layout
    fig, axs = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Motor 3 Control Parameters: With vs Without Spike Filter', fontsize=16)

    # Define plot configurations
    plot_configs = [
        {
            'ax': axs[0, 0],
            'title': 'Potentiometer Raw Values',
            'y_label': 'Raw ADC Value',
            'with_label': 'With Spike Filter',
            'without_label': 'Without Spike Filter',
            'with_color': 'blue',
            'without_color': 'orange',
            'with_data': df_with_filter['raw'],
            'without_data': df_without_filter['raw']
        },
        {
            'ax': axs[0, 1],
            'title': 'Current Angle (Degrees)',
            'y_label': 'Angle (°)',
            'with_label': 'With Spike Filter',
            'without_label': 'Without Spike Filter',
            'with_color': 'green',
            'without_color': 'red',
            'with_data': df_with_filter['angle'],
            'without_data': df_without_filter['angle']
        },
        {
            'ax': axs[1, 0],
            'title': 'Error to Target (Degrees)',
            'y_label': 'Error (°)',
            'with_label': 'With Spike Filter',
            'without_label': 'Without Spike Filter',
            'with_color': 'purple',
            'without_color': 'brown',
            'with_data': df_with_filter['error'],
            'without_data': df_without_filter['error']
        },
        {
            'ax': axs[1, 1],
            'title': 'Control Signal Percentage',
            'y_label': 'Control Signal (%)',
            'with_label': 'With Spike Filter',
            'without_label': 'Without Spike Filter',
            'with_color': 'cyan',
            'without_color': 'magenta',
            'with_data': df_with_filter['control_signal'],
            'without_data': df_without_filter['control_signal']
        }
    ]

    for config in plot_configs:
        ax = config['ax']
        ax.plot(df_with_filter['relative_time'], config['with_data'], label=config['with_label'], color=config['with_color'])
        ax.plot(df_without_filter['relative_time'], config['without_data'], label=config['without_label'], color=config['without_color'])
        ax.set_title(config['title'])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(config['y_label'])
        ax.legend()
        ax.grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig('motor3_control_plot.png')
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
    plot_motor_data(df_with_filter, df_without_filter)
    print("Plot saved as 'motor3_control_plot.png' and displayed.")

if __name__ == "__main__":
    main()
