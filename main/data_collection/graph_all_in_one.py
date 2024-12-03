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

def plot_combined_motor_data(df_with_filter, df_without_filter):
    """
    Plots all parameters in a single graph for each dataset: with spike filter and without spike filter.

    Args:
        df_with_filter (pd.DataFrame): DataFrame with spike filter active.
        df_without_filter (pd.DataFrame): DataFrame with spike filter inactive.
    """
    # Map raw ADC values to degrees
    df_with_filter['raw_mapped'] = df_with_filter['raw'] * (330 / 1023)
    df_without_filter['raw_mapped'] = df_without_filter['raw'] * (330 / 1023)

    # Plot with spike filter
    plt.figure(figsize=(10, 6))
    plt.plot(df_with_filter['relative_time'], df_with_filter['raw_mapped'], label='Raw ADC Value (Degrees)', marker='o')
    plt.plot(df_with_filter['relative_time'], df_with_filter['angle'], label='Angle (°)', linestyle='--')
    plt.plot(df_with_filter['relative_time'], df_with_filter['error'], label='Error (°)', linestyle=':')
    plt.plot(df_with_filter['relative_time'], df_with_filter['control_signal'], label='Control Signal (%)', marker='x')

    plt.title("Motor 3 Control Data (With Spike Filter)")
    plt.xlabel("Time (s)")
    plt.ylabel("Values")
    plt.legend()
    plt.grid(True)
    plt.show()

    # Plot without spike filter
    plt.figure(figsize=(10, 6))
    plt.plot(df_without_filter['relative_time'], df_without_filter['raw_mapped'], label='Raw ADC Value (Degrees)', marker='o')
    plt.plot(df_without_filter['relative_time'], df_without_filter['angle'], label='Angle (°)', linestyle='--')
    plt.plot(df_without_filter['relative_time'], df_without_filter['error'], label='Error (°)', linestyle=':')
    plt.plot(df_without_filter['relative_time'], df_without_filter['control_signal'], label='Control Signal (%)', marker='x')

    plt.title("Motor 3 Control Data (Without Spike Filter)")
    plt.xlabel("Time (s)")
    plt.ylabel("Values")
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    # Define log file paths
    log_with_filter = 'motor3_with_spike_filter_1.log'
    log_without_filter = 'motor3_without_spike_filter_1.log'

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
    print("Plotting combined graphs...")
    plot_combined_motor_data(df_with_filter, df_without_filter)
    print("Plots saved and displayed.")

if __name__ == "__main__":
    main()
