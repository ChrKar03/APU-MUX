import os
import sys
import time
import signal
import serial
import pyvisa
import numpy as np
import pandas as pd
import serial.tools.list_ports
import matplotlib.pyplot as plt

# Raw data directory
RAW_DATA_DIR = "RAW_DATA/"

# STM32 Settings
# SAMPLE_CLK = 17e7             # 170 MHz
SAMPLE_CLK = 42.5e7             # 42.5 MHz
SAMPLES_BUF_SIZE = 12500        # Size of the sample buffer (max 12500 for STM32G4)

#STM32 UART Settings
SER = None
BAUDRATE = 115200
SER_TIMEOUT = 1
SERIAL_PORT = None

# Agilent AWG Settings
AWG = None
DUTY_VALUE = [5, 10, 20, 30, 40, 50, 60, 70, 80, 90]

# Frequency Sweep Settings
FREQ_LIST = np.array([1e3, 1e4, 1e5, 1e6, 1e7])
FREQ_TO_TICKS = [int(SAMPLE_CLK / f) for f in FREQ_LIST]

# Data Buffers
FREQ_BUF = np.zeros((len(FREQ_LIST), 12500))
DUTY_BUF = np.zeros((len(FREQ_LIST), 12500))
ERROR_BUF = np.zeros((len(FREQ_LIST), 12500))

# Helper Functions
def find_stm32_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # port.device -> e.g. '/dev/ttyACM0'
        # port.manufacturer -> e.g. 'STMicroelectronics'
        # port.description -> e.g. 'STM32 STLink Virtual COM Port'
        if port.manufacturer and "STMicroelectronics" in port.manufacturer:
            return port.device
    return None

def find_agilent_awg():
    rm = pyvisa.ResourceManager('@py')
    instruments = rm.list_resources()
    for inst in instruments:
        if "USB" in inst:
            print(f"Found Agilent AWG: {inst}")
            return rm.open_resource(inst)
    return None

def init_comms():
    global SER, AWG
    SERIAL_PORT = find_stm32_port()
    if not SERIAL_PORT:
        print("STM32 not found.")
        return False
    print(f"Found STM32 on port: {SERIAL_PORT}")
    SER = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=SER_TIMEOUT)

    AWG = find_agilent_awg()
    if not AWG:
        print("No Agilent AWG found.")
        return False
    return True

# Set PWM on Agilent AWG
def set_pwm(frequency_hz, duty_cycle_percent):
    AWG.write(f':SOURce:FUNCtion SQUare')
    time.sleep(0.2)
    AWG.write(f':SOURce:FREQuency {frequency_hz}')
    time.sleep(0.2)
    AWG.write(f':SOURce:VOLTage:AMPLitude 5')
    time.sleep(0.2)
    AWG.write(f':SOURce:VOLTage:OFFSet 0')
    time.sleep(0.2)
    AWG.write(f':SOURce:PHASe 0')
    time.sleep(0.2)
    AWG.write(f':SOURce:FUNCtion:SQUare:DUTY {duty_cycle_percent}')
    time.sleep(0.2)

# Read from UART until terminator is found
def read_until_terminator(terminator=b'END'):
    buffer = b""
    while terminator not in buffer:
        chunk = SER.read(1024)  # Read 1024 bytes
        if not chunk:
            continue
        buffer += chunk
        print(f"Received {len(buffer)} bytes", end='\r')

    # Ignore any data after the terminator
    buffer = buffer[:buffer.find(terminator) + len(terminator)]

    print()
    data = buffer.split(terminator)[0]
    if len(data) % 8 != 0:
        raise ValueError("Data not aligned to 8-byte measurement boundaries")

    return np.frombuffer(data, dtype='<u4')

# Frequency and Duty Cycle Test
def freq_duty_test():
    # Enable AWG output
    AWG.write(':SOURce:OUTPut:STATe 1')

    for i in range(len(FREQ_LIST)):
        for j in range(len(DUTY_VALUE)):
            print(f"Setting frequency: {int(FREQ_LIST[i])} Hz, Duty Cycle: {DUTY_VALUE[j]}%")
            set_pwm(int(FREQ_LIST[i]), DUTY_VALUE[j])

            # Start data capture
            SER.flushInput()
            SER.flushOutput()
            time.sleep(0.5)
            SER.write(b'START')
            time.sleep(0.5)  # Wait for the signal to stabilize
            try:
                data = read_until_terminator()  # returns array of uint32
                if len(data) % 2 != 0:
                    print("Warning: Unexpected data length")
                    continue

                for k in range(0, len(data), 2):
                    idx = k // 2
                    FREQ_BUF[i, idx] = data[k]
                    DUTY_BUF[i, idx] = data[k+1]

                    # FREQ_BUF[i, idx], DUTY_BUF[i, idx] = data[j:j+2]
                data = None

                with open(f"{RAW_DATA_DIR}f_{int(FREQ_LIST[i])}_d_{DUTY_VALUE[j]}.csv", "w") as f:
                    f.write("Period (ticks),Duty Cycle (ticks)\n")
                    for k in range(len(FREQ_BUF[i])):
                        if FREQ_BUF[i, k] == 0 and DUTY_BUF[i, k] == 0:
                            break
                        f.write(f"{FREQ_BUF[i, k]},{DUTY_BUF[i, k]}\n")
            except ValueError as e:
                print(f"Error reading UART data: {e}")
            except Exception as e:
                print(f"General error: {e}")

    # Disable AWG output
    AWG.write(':SOURce:OUTPut:STATe 0')    

# Graceful exit on Ctrl+C
def signal_handler(sig, frame):
    print("\nInterrupt received. Closing connections...")
    if AWG:
        AWG.write(':SOURce:OUTPut:STATe 0')
        AWG.close()
    if SER:
        SER.close()
    sys.exit(0)

def signal_handler2(sig, frame):
    raise Exception("SIGHUP received, continuing...")

if __name__ == "__main__":
    # Clear directories
    for filename in os.listdir(RAW_DATA_DIR):
        file_path = os.path.join(RAW_DATA_DIR, filename)
        try:
            if os.path.isfile(file_path):
                os.unlink(file_path)
        except Exception as e:
            print(f"Error deleting file {file_path}: {e}")

    # # Initialize communications
    if not init_comms():
        exit(1)

    # signal.signal(signal.SIGINT, signal_handler)

    # # PWM freq and duty cycle sampling test
    print("Starting frequency and duty cycle test...")
    freq_duty_test()

    print("Done.")
    print("Closing connections...")

    # Close the Agilent AWG and serial connections
    if AWG:
        AWG.write(':SOURce:OUTPut:STATe 0')
        AWG.close()
    if SER:
        SER.close()