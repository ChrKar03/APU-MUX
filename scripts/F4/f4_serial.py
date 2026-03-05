import sys
import time
import signal
import serial
import serial.tools.list_ports
import struct

#STM32 UART Settings
SER = None
BAUDRATE = 115200
SER_TIMEOUT = 1
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
    SER = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=SER_TIMEOUT)

    return True

# Graceful exit on Ctrl+C
def signal_handler(sig, frame):
    print("\nInterrupt received. Closing connections...")
    if SER:
        SER.close()
    sys.exit(0)

def signal_handler2(sig, frame):
    raise Exception("SIGHUP received, continuing...")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGHUP, signal_handler2)

    if not init_comms():
        sys.exit(1)

    print("Entering main loop. Press Ctrl+C to exit.")
    while True:
        try:
            data = input("Enter value for PWM duty cycle (500-10000) or 'exit': ")
            if data.lower() == 'exit':
                break
            
            value = int(data)
            if 500 <= value <= 10000:
                # Create a list of 8 identical PWM values
                pwm_array = [value] * 8 
                
                # Pack the 8 values into a 16-byte string.
                # '<' means Little-Endian (STM32 format)
                # '8H' means 8 Unsigned 16-bit Integers (uint16_t)
                payload = struct.pack('<8H', *pwm_array)
                
                # Send all 16 bytes at once
                SER.write(payload)
                print(f"Sent 16 bytes: {payload.hex()}")
                
            else:
                print("Value out of range (500-10000).")
                
        except ValueError:
            print("Invalid input. Please enter a number.")
        except Exception as e:
            print(f"Error: {e}")
            break

    if SER:
        SER.close()