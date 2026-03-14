import pandas as pd
import struct
import matplotlib.pyplot as plt
import os
import sys

def decode_motor_hex(hex_str):
    """
    Gazebo FDM motor commands are sent as an array of 32-bit (4-byte) floats.
    Each float takes up 8 hexadecimal characters.
    We will extract the first 4 motors (Quadcopter).
    """
    try:
        if not isinstance(hex_str, str) or len(hex_str) < 32:
            return pd.Series([None, None, None, None])
        
        motors = []
        for i in range(4):
            chunk = hex_str[i*8 : (i+1)*8]
            float_val = struct.unpack('<f', bytes.fromhex(chunk))[0]
            motors.append(float_val)
            
        return pd.Series(motors)
    except Exception as e:
        return pd.Series([None, None, None, None])

def run_analysis(file_path=None):
    if file_path is None:
        file_path = 'sitl_analysis.csv'
    if not os.path.exists(file_path):
        print(f"Error: Could not find {file_path}. Have you run the simulation yet?")
        return

    print("Loading and decoding data (this might take a few seconds)...")
    
    df = pd.read_csv(file_path, names=['Timestamp', 'SITL', 'HexData'])
    df['Timestamp'] = pd.to_datetime(df['Timestamp'], format='%H:%M:%S.%f', errors='coerce')
    df[['Motor1', 'Motor2', 'Motor3', 'Motor4']] = df['HexData'].apply(decode_motor_hex)
    df = df.dropna(subset=['Motor1'])

    sitl1_data = df[df['SITL'] == 'SITL_1'].copy()
    sitl2_data = df[df['SITL'] == 'SITL_2'].copy()

    merged = pd.merge_asof(
        sitl1_data.sort_values('Timestamp'), 
        sitl2_data.sort_values('Timestamp'), 
        on='Timestamp', 
        direction='nearest',
        suffixes=('_1', '_2')
    )

    print("\n" + "="*60)
    print("STATISTICAL SUMMARY & DIVERGENCE (SITL 1 vs SITL 2)")
    print("="*60)

    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle('PID Controller Drift: Motor Throttle Outputs Over Time', fontsize=16)

    for i in range(1, 5):
        motor = f'Motor{i}'
        
        print(f"\n--- {motor.upper()} ---")
        merged[f'{motor}_Error'] = abs(merged[f'{motor}_1'] - merged[f'{motor}_2'])
        
        avg_drift = merged[f'{motor}_Error'].mean()
        max_drift = merged[f'{motor}_Error'].max()
        
        print(f"SITL 1 Mean: {sitl1_data[motor].mean():.4f} | SITL 2 Mean: {sitl2_data[motor].mean():.4f}")
        print(f"Average Divergence: {avg_drift:.4f}")
        print(f"Maximum Divergence: {max_drift:.4f}")

        row = (i - 1) // 2
        col = (i - 1) % 2
        ax = axes[row, col]
        
        ax.plot(sitl1_data['Timestamp'], sitl1_data[motor], label=f'SITL 1', alpha=0.8, color='blue')
        ax.plot(sitl2_data['Timestamp'], sitl2_data[motor], label=f'SITL 2', alpha=0.8, color='red', linestyle='--')
        
        ax.set_title(f'{motor} Comparison')
        ax.set_xlabel('Time')
        ax.set_ylabel('Throttle Output (0.0 to 1.0)')
        ax.legend()
        ax.grid(True)

    plt.tight_layout()
    plt.subplots_adjust(top=0.92) 
    
    print("\nGenerating 2x2 plot grid...")
    plt.savefig('pid_drift_analysis_all_motors.png')
    plt.show()

if __name__ == "__main__":
    run_analysis(sys.argv[1])