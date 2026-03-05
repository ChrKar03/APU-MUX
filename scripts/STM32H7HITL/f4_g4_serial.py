import sys
import time
import signal
import serial
import serial.tools.list_ports
import struct
import threading

# ---------------------------------------------------------
# 1. HARDWARE CONFIGURATION
# ---------------------------------------------------------
# Replace these strings with the actual Serial Numbers of your boards.
# Run 'python -m serial.tools.list_ports -v' to find them.
BOARD_MAP = {
    "READER_BOARD": "003E00263433510237363934", 
    "WRITER_BOARD": "066EFF3733474B3043022227"   
}

BAUDRATE = 115200
TIMEOUT = 1

# Global Handles
SER_READ = None
SER_WRITE = None
RUNNING = True

# ---------------------------------------------------------
# 2. CONNECTION & SETUP
# ---------------------------------------------------------
def init_comms():
    global SER_READ, SER_WRITE
    
    print("[HW] Scanning for STM32 boards...")
    ports = serial.tools.list_ports.comports()
    found_reader = False
    found_writer = False

    for p in ports:
        # Check for Reader Board (Script 1 source)
        if BOARD_MAP["READER_BOARD"] in (p.serial_number or ""):
            try:
                SER_READ = serial.Serial(p.device, BAUDRATE, timeout=TIMEOUT)
                print(f" + [READER] Found at {p.device}")
                found_reader = True
                SER_READ.reset_input_buffer()
            except Exception as e:
                print(f" ! [READER ERR] Found {p.device} but failed to open: {e}")

        # Check for Writer Board (Script 2 destination)
        elif BOARD_MAP["WRITER_BOARD"] in (p.serial_number or ""):
            try:
                SER_WRITE = serial.Serial(p.device, BAUDRATE, timeout=TIMEOUT)
                print(f" + [WRITER] Found at {p.device}")
                found_writer = True
            except Exception as e:
                print(f" ! [WRITER ERR] Found {p.device} but failed to open: {e}")
        
        else:
            # Helper to identify other devices
            if "STM" in (p.manufacturer or ""):
                print(f" ? [IGNORED STM] Device: {p.device} | Serial: {p.serial_number}")

    return found_reader and found_writer

# ---------------------------------------------------------
# 3. BACKGROUND THREAD: READING (Script 1 Logic)
# ---------------------------------------------------------
def read_task():
    """
    Continuously reads 4x uint16_t from the READER board.
    Runs in a background thread.
    """
    global RUNNING
    print("[THREAD] Read loop started...")
    
    while RUNNING:
        if SER_READ and SER_READ.is_open:
            try:
                # Wait for 8 bytes (4 * uint16)
                if SER_READ.in_waiting >= 8:
                    data = SER_READ.read(8)
                    
                    if len(data) == 8:
                        # Unpack 4 Little-Endian unsigned shorts
                        unpacked_values = struct.unpack('<4H', data)
                        
                        # Print with a carriage return to avoid spamming the console 
                        # while the user is trying to type
                        sys.stdout.write(f"\r[R] Received: {unpacked_values} | Bytes: {data.hex()}   ")
                        sys.stdout.flush()
                    else:
                        SER_READ.reset_input_buffer()
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"\n[READ ERROR] {e}")
                break
        else:
            time.sleep(0.1)

# ---------------------------------------------------------
# 4. MAIN LOOP: WRITING (Script 2 Logic)
# ---------------------------------------------------------
def signal_handler(sig, frame):
    global RUNNING
    print("\n\nInterrupt received. Closing connections...")
    RUNNING = False
    if SER_READ: SER_READ.close()
    if SER_WRITE: SER_WRITE.close()
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    # 1. Connect
    if not init_comms():
        print("\n[ERROR] Could not find both boards. Check Serial Numbers in BOARD_MAP.")
        # We don't exit here immediately to allow testing if you only have one board,
        # but logically the script needs both.
        
        # Uncomment the line below to enforce strict checking:
        # sys.exit(1) 

    # 2. Start Reading Thread
    if SER_READ:
        t_read = threading.Thread(target=read_task, daemon=True)
        t_read.start()

    # 3. Enter Writing Loop
    print("\n" + "="*50)
    print("COMMAND CONSOLE")
    print("Enter PWM duty cycle (500-10000) or 'exit'.")
    print("Incoming data will appear with [R] prefix.")
    print("="*50 + "\n")

    while RUNNING:
        try:
            # We use a blank input prompt so it doesn't fight with the print statements
            user_input = input() 
            
            if user_input.lower() == 'exit':
                RUNNING = False
                break
            
            if not user_input.strip():
                continue

            try:
                value = int(user_input)
                if 500 <= value <= 10000:
                    if SER_WRITE:
                        # Create list of 8 identical values
                        pwm_array = [value] * 4 + [1000] * 4
                        # Pack 8 uint16 (Little-Endian)
                        payload = struct.pack('<8H', *pwm_array)
                        
                        SER_WRITE.write(payload)
                        print(f"\n[W] Sent PWM {value} (Hex: {payload.hex()})")
                    else:
                        print("\n[W] Error: Writer board not connected.")
                else:
                    print("\n[CMD] Value out of range (500-10000).")
            except ValueError:
                print("\n[CMD] Invalid input. Please enter a number.")

        except Exception as e:
            print(f"\n[MAIN ERROR] {e}")
            break

    if SER_READ: SER_READ.close()
    if SER_WRITE: SER_WRITE.close()
    print("Bye.")