import os
import sys
import time
import signal
import serial
import numpy as np
import pandas as pd
import serial.tools.list_ports
import matplotlib.pyplot as plt

#STM32 UART Settings
SER = None
BAUDRATE = 115200
# SER_TIMEOUT = 1
SERIAL_PORT = None

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

def init_comms():
    global SER, AWG
    SERIAL_PORT = find_stm32_port()
    if not SERIAL_PORT:
        print("STM32 not found.")
        return False
    print(f"Found STM32 on port: {SERIAL_PORT}")
    SER = serial.Serial(SERIAL_PORT, BAUDRATE)#, timeout=SER_TIMEOUT)

    return True

# Graceful exit on Ctrl+C
def signal_handler(sig, frame):
    print("\nInterrupt received. Closing connections...")
    if SER:
        SER.close()
    sys.exit(0)

if __name__ == "__main__":
    if not init_comms():
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)
    print("Communications initialized.")

    with open("input_dc", "r") as f:
        with open("output_dc", "w") as d:
            for _ in range(0, 100, 4):
                # read 4 lines, parse ints and pack into 4 little-endian 32-bit values
                vals = []
                for _ in range(4):
                    line = f.readline()
                    try:
                        vals.append(int(line.strip()))
                    except Exception:
                        vals.append(0)
                # build byte array (4 bytes per integer, little-endian)
                b = bytearray()
                for v in vals:
                    # use signed if negative, unsigned otherwise
                    if v < 0:
                        b += v.to_bytes(4, "little", signed=True)
                    else:
                        b += v.to_bytes(4, "little", signed=False)
                input_duty = bytes(b)

                SER.write(input_duty)
                output_duty = SER.read(4*4)
                for i in range(0, 4, 4):
                    d.write(int(output_duty[i:4+i]))
                    d.write("\n")
    
    SER.close()
    print("Data acquisition complete. Output written to 'output_dc'.")

    with open("input_dc", "r") as f:
        with open("output_dc", "r") as d:
            for _ in range(0, 100):
                in_line = f.readline()
                out_line = d.readline()
                try:
                    in_dc = int(in_line.strip())
                    out_dc = int(out_line.strip())
                    diff = abs(in_dc - out_dc)
                except Exception:
                    diff = 0
                if (diff > 5):
                    print(f"Mismatch between input ({in_dc}) and output ({out_dc}) duty cycle files.")
                    print("Exiting...")
                    sys.exit(1)
