import pandas as pd
import struct
import matplotlib.pyplot as plt
import os

def decode_tx(hex_str):
    """Decodes the 32-character Hex string from SITL TX into 8 uint16s"""
    try:
        if not isinstance(hex_str, str) or len(hex_str) < 32:
            return pd.Series([None]*8)
        return pd.Series(struct.unpack('<8H', bytes.fromhex(hex_str)))
    except Exception:
        return pd.Series([None]*8)

def decode_rx(hex_str):
    """Decodes the 16-character Hex string from STM32 RX into 4 uint16s"""
    try:
        if not isinstance(hex_str, str) or len(hex_str) < 16:
            return pd.Series([None]*4)
        return pd.Series(struct.unpack('<4H', bytes.fromhex(hex_str)))
    except Exception:
        return pd.Series([None]*4)

def run_analysis():
    file_path = './sitl_analysis.csv'
    if not os.path.exists(file_path):
        print(f"Error: Could not find {file_path}.")
        return

    print("Loading serial log data...")
    df = pd.read_csv(file_path, names=['Timestamp', 'Type', 'Hex'])
    df['Timestamp'] = pd.to_datetime(df['Timestamp'], format='%H:%M:%S.%f', errors='coerce')

    # Separate TX and RX data
    tx_df = df[df['Type'] == 'SERIAL_TX'].copy()
    rx_df = df[df['Type'] == 'SERIAL_RX'].copy()

    print("Decoding binary payloads...")
    tx_cols = ['S1_M1', 'S1_M2', 'S1_M3', 'S1_M4', 'S2_M1', 'S2_M2', 'S2_M3', 'S2_M4']
    rx_cols = ['VOTED_M1', 'VOTED_M2', 'VOTED_M3', 'VOTED_M4']
    
    tx_df[tx_cols] = tx_df['Hex'].apply(decode_tx)
    rx_df[rx_cols] = rx_df['Hex'].apply(decode_rx)

    merged = pd.merge_asof(
        rx_df.sort_values('Timestamp'), 
        tx_df.sort_values('Timestamp'), 
        on='Timestamp', 
        direction='nearest'
    )

    print("Generating Hardware-in-the-Loop plots...")
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle(f'STM32H7 Output vs SITL Input', fontsize=16)

    for i in range(1, 5):
        row = (i - 1) // 2
        col = (i - 1) % 2
        ax = axes[row, col]
        
        ax.plot(merged['Timestamp'], merged[f'S1_M{i}'], label=f'SITL 1 (Input)', alpha=0.6, color='blue')
        ax.plot(merged['Timestamp'], merged[f'S2_M{i}'], label=f'SITL 2 (Input)', alpha=0.6, color='green')
        ax.plot(merged['Timestamp'], merged[f'VOTED_M{i}'], label=f'STM32H7 (Output)', alpha=1.0, color='red', linestyle='--')
        
        ax.set_title(f'Motor {i} PWM')
        ax.set_xlabel('Time')
        ax.set_ylabel('Mapped PWM (0-2000)')
        ax.legend()
        ax.grid(True)

    plt.tight_layout()
    plt.subplots_adjust(top=0.92) 
    plt.savefig('stm32hitl_analysis.png')
    plt.show()

if __name__ == "__main__":
    run_analysis()