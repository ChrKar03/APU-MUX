import sys
import time
import signal
import serial
import serial.tools.list_ports
import struct
import threading
import statistics

# ---------------------------------------------------------
# 1. HARDWARE CONFIGURATION
# ---------------------------------------------------------
BOARD_MAP = {
    "READER_BOARD": "003E00263433510237363934", # G4
    "WRITER_BOARD": "066EFF3733474B3043022227"  # F4 
}

BAUDRATE = 115200

# Global Handles & State
SER_READ = None
SER_WRITE = None
RUNNING = True

# --- LATENCY TRACKING GLOBALS ---
WAITING_FOR_REPLY = False
START_TIME = 0.0
EXPECTED_PWM = 0
PWM_TOLERANCE = 1  
LATENCIES = []
# ------------------------------------

def init_comms():
    global SER_READ, SER_WRITE
    print("[HW] Scanning for STM32 boards...")
    ports = serial.tools.list_ports.comports()
    found_reader = False
    found_writer = False

    for p in ports:
        if BOARD_MAP["READER_BOARD"] in (p.serial_number or ""):
            try:
                SER_READ = serial.Serial(p.device, BAUDRATE)
                print(f" + [READER] Found at {p.device}")
                found_reader = True
                SER_READ.reset_input_buffer()
            except Exception as e:
                print(f" ! [READER ERR] Failed to open: {e}")

        elif BOARD_MAP["WRITER_BOARD"] in (p.serial_number or ""):
            try:
                SER_WRITE = serial.Serial(p.device, BAUDRATE)
                print(f" + [WRITER] Found at {p.device}")
                found_writer = True
            except Exception as e:
                print(f" ! [WRITER ERR] Failed to open: {e}")
    
    return found_reader and found_writer

# ---------------------------------------------------------
# 3. BACKGROUND THREAD: READING (G4 -> Python)
# ---------------------------------------------------------
def read_task():
    global RUNNING, WAITING_FOR_REPLY, START_TIME, EXPECTED_PWM, LATENCIES
    print("[THREAD] Read loop started...")
    
    MAGIC_HEADER = b'\xaa\xbb'
    
    while RUNNING:
        if SER_READ and SER_READ.is_open:
            try:
                header_data = SER_READ.read_until(MAGIC_HEADER)
                
                data = SER_READ.read(8)
                
                if len(data) == 8:
                    unpacked_values = struct.unpack('<4H', data)
                    
                    if WAITING_FOR_REPLY and abs(unpacked_values[0] - EXPECTED_PWM) <= PWM_TOLERANCE:
                        end_time = time.perf_counter()
                        duration_ms = (end_time - START_TIME) * 1000
                        LATENCIES.append(duration_ms)
                        
                        sys.stdout.write(f"\r[INFO] Iteration {len(LATENCIES)}/10000 | RTT: {duration_ms:.2f} ms | Read: {unpacked_values}   ")
                        sys.stdout.flush()
                        WAITING_FOR_REPLY = False
                        
            except Exception as e:
                print(f"\n[READ ERROR] {e}")
                break
        else:
            time.sleep(0.1)

# ---------------------------------------------------------
# 4. MAIN LOOP: AUTOMATED SWEEP (Python -> F4)
# ---------------------------------------------------------
def signal_handler(sig, frame):
    global RUNNING
    print("\n\nInterrupt received. Closing connections...")
    RUNNING = False
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    if not init_comms():
        print("\n[WARNING] One or more boards missing.")

    if SER_READ:
        t_read = threading.Thread(target=read_task, daemon=True)
        t_read.start()

    ITERATIONS = 10000

    print("\n" + "="*50)
    print(f"AUTOMATED HITL LATENCY BENCHMARK ({ITERATIONS} Iters)")
    print("="*50 + "\n")
    
    # Small pause to let the read thread initialize
    time.sleep(1)

    for i in range(ITERATIONS):
        if not RUNNING:
            break

        if SER_WRITE:
            sweep_value = 1100 + (i % 800)
            pwm_array = [sweep_value] * 8
            payload_sweep = struct.pack('<8H', *pwm_array)
            
            EXPECTED_PWM = sweep_value
            START_TIME = time.perf_counter()
            WAITING_FOR_REPLY = True
            
            SER_WRITE.write(payload_sweep)

            timeout_counter = 0
            while WAITING_FOR_REPLY and RUNNING:
                time.sleep(0.001)
                timeout_counter += 1
                if timeout_counter > 200:
                    print(f"\n[WARN] Timeout waiting for value {sweep_value}. Frame lost?")
                    WAITING_FOR_REPLY = False
                    break

    # ---------------------------------------------------------
    # 5. PRINT STATISTICS AND EXIT
    # ---------------------------------------------------------
    RUNNING = False

    print("\n\n" + "="*50)
    print("HITL PROPAGATION LATENCY STATISTICS")
    print("="*50)
    if len(LATENCIES) > 0:
        print(f"Total Valid Samples : {len(LATENCIES)}")
        print(f"Minimum RTT         : {min(LATENCIES):.2f} ms")
        print(f"Maximum RTT         : {max(LATENCIES):.2f} ms")
        print(f"Average RTT         : {statistics.mean(LATENCIES):.2f} ms")
        if len(LATENCIES) > 1:
            print(f"Standard Deviation  : {statistics.stdev(LATENCIES):.2f} ms")
    else:
        print("No valid data collected. Check connections.")
    print("="*50 + "\n")

    if SER_READ: SER_READ.close()
    if SER_WRITE: SER_WRITE.close()
    sys.exit(0)