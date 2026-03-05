import sys
import time
import signal
import serial
import serial.tools.list_ports
import struct

# STM32 UART Settings
SER = None
BAUDRATE = 115200
SER_TIMEOUT = 1
SERIAL_PORT = None

# Helper Functions
def find_stm32_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.manufacturer and "STMicroelectronics" in port.manufacturer:
            return port.device
    return None

def init_comms():
    global SER
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

    print("Entering read loop. Press Ctrl+C to exit.")
    
    # Clear the buffer before starting to ensure we don't read stale/misaligned data
    SER.reset_input_buffer()

    while True:
        try:
            # Wait until we have at least 8 bytes (4 * uint16_t) in the incoming buffer
            if SER.in_waiting >= 8:
                # Read exactly 8 bytes
                data = SER.read(8)
                
                if len(data) == 8:
                    # Unpack the 8 bytes into 4 Little-Endian unsigned 16-bit integers
                    # '<' means Little-Endian
                    # '4H' means 4 Unsigned Shorts (uint16_t)
                    unpacked_values = struct.unpack('<4H', data)
                    
                    print(f"Received values: {unpacked_values} | Raw bytes: {data.hex()}")
                else:
                    # If we somehow read less than 8 bytes, clear the buffer to realign
                    print("Warning: Partial packet received. Realigning...")
                    SER.reset_input_buffer()
            else:
                # Small sleep to prevent the loop from hogging 100% of the CPU
                time.sleep(0.01)

        except Exception as e:
            print(f"Error: {e}")
            break

    if SER:
        SER.close()