import sys
import time
import signal
import serial
import serial.tools.list_ports
import struct
import threading
import socket
from pymavlink import mavutil, mavwp
from concurrent.futures import ThreadPoolExecutor

# ---------------------------------------------------------
# 1. HARDWARE & NETWORK CONFIGURATION
# ---------------------------------------------------------
BOARD_MAP = {
    "READER_BOARD": "003E00263433510237363934", # G4 Board (Reads 4x uint16)
    "WRITER_BOARD": "066EFF3733474B3043022227"  # F4 Board (Writes 8x uint16)
}

BAUDRATE = 115200
TIMEOUT = 1

# Gazebo FDM Ports
GAZEBO_LISTEN_PORT = 9002  #FROM Gazebo
GAZEBO_TARGET_IP = "127.0.0.1"
GAZEBO_TARGET_PORT = 9003  #BTO Gazebo

# SITL 1 FDM Ports
SITL1_LISTEN_PORT = 9012   #FROM SITL 1 
SITL1_PHYSICS_IN = ("127.0.0.1", 9013) #TO SITL 1

# SITL 2 FDM Ports
SITL2_LISTEN_PORT = 9022   #FROM SITL 2
SITL2_PHYSICS_IN = ("127.0.0.1", 9023) #TO SITL 2

MAVLINK_SOURCE1 = "udpin:127.0.0.1:14550"
MAVLINK_SOURCE2 = "udpin:127.0.0.1:14560"

# ---------------------------------------------------------
# 2. DUAL SITL HARDWARE BRIDGE CLASS
# ---------------------------------------------------------
class DualSITLHardwareBridge:
    def __init__(self):
        self.running = False
        self.debug_mode = False  
        self.uploading_mission = False 
        
        self.ser_read = None
        self.ser_write = None

        # State to hold the 8 uint16 values for the F4 board
        # Indices 0-3 for SITL1, 4-7 for SITL2
        self.sitl_outputs = [1000] * 8 
        self.sitl_lock = threading.Lock()

        self._setup_serial()
        self._setup_sockets()
        self._setup_mavlink()

    def _setup_serial(self):
        print("[HW] Scanning for STM32 boards...")
        ports = serial.tools.list_ports.comports()
        
        for p in ports:
            if BOARD_MAP["READER_BOARD"] in (p.serial_number or ""):
                try:
                    self.ser_read = serial.Serial(p.device, BAUDRATE, timeout=TIMEOUT)
                    print(f" + [READER (G4)] Found at {p.device}")
                    self.ser_read.reset_input_buffer()
                except Exception as e:
                    print(f" ! [READER ERR] Failed to open {p.device}: {e}")

            elif BOARD_MAP["WRITER_BOARD"] in (p.serial_number or ""):
                try:
                    self.ser_write = serial.Serial(p.device, BAUDRATE, timeout=TIMEOUT)
                    print(f" + [WRITER (F4)] Found at {p.device}")
                except Exception as e:
                    print(f" ! [WRITER ERR] Failed to open {p.device}: {e}")

        if not self.ser_read or not self.ser_write:
            print("\n[WARNING] Could not connect to both STM boards. Bridge will still route UDP where possible.")

    def _setup_sockets(self):
        try:
            print("[NET] Setting up FDM Physics routing...")
            self.sock_gazebo_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock_gazebo_in.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock_gazebo_in.bind(("127.0.0.1", GAZEBO_LISTEN_PORT))

            self.sock_sitl1_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock_sitl1_in.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock_sitl1_in.bind(("127.0.0.1", SITL1_LISTEN_PORT))

            self.sock_sitl2_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock_sitl2_in.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock_sitl2_in.bind(("127.0.0.1", SITL2_LISTEN_PORT))

            self.sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            self.sock_sitl1_in.settimeout(0.1)
            self.sock_sitl2_in.settimeout(0.1)
        except Exception as e:
            print(f"Socket setup failed: {e}")
            sys.exit(1)

    def _setup_mavlink(self):
        try:
            print("[MAV] Connecting to MAVLink streams...")
            self.mav_conn1 = mavutil.mavlink_connection(MAVLINK_SOURCE1)
            self.mav_conn2 = mavutil.mavlink_connection(MAVLINK_SOURCE2)
        except Exception as e:
            print(f"MAVLink setup failed: {e}")
            sys.exit(1)

    # ---------------------------------------------------------
    # 3. THREADS
    # ---------------------------------------------------------
    def start(self):
        self.running = True
        
        # Hardware & Physics Threads
        threading.Thread(target=self.thread_gazebo_to_sitls, daemon=True).start()
        threading.Thread(target=self.thread_sitl_to_writer, args=(self.sock_sitl1_in, True), daemon=True).start()
        threading.Thread(target=self.thread_sitl_to_writer, args=(self.sock_sitl2_in, False), daemon=True).start()
        threading.Thread(target=self.thread_reader_to_gazebo, daemon=True).start()
        
        # MAVLink Threads
        threading.Thread(target=self.thread_mavlink_monitor, args=(self.mav_conn1, "SITL 1"), daemon=True).start()
        threading.Thread(target=self.thread_mavlink_monitor, args=(self.mav_conn2, "SITL 2"), daemon=True).start()

        self.print_menu()

    def thread_gazebo_to_sitls(self):
        """Routes Physics from Gazebo directly to BOTH SITL instances."""
        while self.running:
            try:
                data, addr = self.sock_gazebo_in.recvfrom(2048)
                self.sock_out.sendto(data, SITL1_PHYSICS_IN)
                self.sock_out.sendto(data, SITL2_PHYSICS_IN)
            except Exception as e:
                if self.running: print(f"Gazebo In Error: {e}")

    def thread_sitl_to_writer(self, sock, is_sitl1):
        """
        Reads 4x 32-bit floats from a SITL, converts [-1, 1] to [0, 2000],
        updates the combined 8-integer array, and writes to F4.
        """
        sitl_name = "SITL 1" if is_sitl1 else "SITL 2"
        while self.running:
            try:
                data, addr = sock.recvfrom(1024)
                
                # We expect at least 16 bytes (4x 32-bit floats)
                if len(data) >= 16:
                    # 1. Unpack 4 floats (Little-Endian)
                    floats = struct.unpack('<4f', data[:16])
                    
                    # 2. Convert from [-1.0, 1.0] to [0, 2000]
                    # Formula: (val + 1.0) * 1000, clamped between 0 and 2000
                    ints = [int(max(0, min(2000, (f + 1.0) * 1000))) for f in floats]
                    
                    # 3. Safely update the shared 8-value array and send to Writer
                    with self.sitl_lock:
                        if is_sitl1:
                            self.sitl_outputs[0:4] = ints
                        else:
                            self.sitl_outputs[4:8] = ints
                        
                        # Pack 8x uint16_t (Little-Endian)
                        payload = struct.pack('<8H', *self.sitl_outputs)
                        
                    # Send payload to STM32
                    if self.ser_write and self.ser_write.is_open:
                        self.ser_write.write(payload)
                        
                        if self.debug_mode:
                            print(f"[HW W] {sitl_name} updated F4: {self.sitl_outputs}")
                            
            except socket.timeout:
                continue
            except Exception as e:
                if self.running: print(f"SITL ({sitl_name}) Handler Error: {e}")

    def thread_reader_to_gazebo(self):
        """
        Reads 4x uint16_t from G4, converts [0, 2000] to [-1, 1],
        and sends raw 4x 32-bit floats to Gazebo.
        """
        print("[THREAD] Hardware Read loop started...")
        while self.running:
            if self.ser_read and self.ser_read.is_open:
                try:
                    if self.ser_read.in_waiting >= 8:
                        data = self.ser_read.read(8)
                        
                        if len(data) == 8:
                            # 1. Unpack 4x uint16_t
                            unpacked_pwms = struct.unpack('<4H', data)
                            
                            # 2. Convert from [0, 2000] to [-1.0, 1.0]
                            # Formula: (val / 2000.0) - 1.0, clamped between -1.0 and 1.0
                            normalized = [max(-1.0, min(1.0, (val / 2000.0) - 1.0)) for val in unpacked_pwms]
                            
                            # 3. Pack as 4x 32-bit floats and send to Gazebo
                            gazebo_payload = struct.pack('<4f', *normalized)
                            self.sock_out.sendto(gazebo_payload, (GAZEBO_TARGET_IP, GAZEBO_TARGET_PORT))
                            
                            if self.debug_mode:
                                print(f"[HW R] G4 -> Gazebo: Raw {unpacked_pwms} | Norm {normalized}")
                        else:
                            self.ser_read.reset_input_buffer()
                    else:
                        time.sleep(0.005)
                except Exception as e:
                    print(f"\n[READ ERROR] {e}")
                    time.sleep(1)
            else:
                time.sleep(0.1)

    # ---------------------------------------------------------
    # 4. MAVLINK & UTILITY FUNCTIONS
    # ---------------------------------------------------------
    def print_menu(self):
        print("\n" + "="*60)
        print("DUAL SITL HARDWARE IN-THE-LOOP BRIDGE")
        print("="*60)
        print(" (m) Load and Upload Mission File (.txt)")
        print(" (s) Synchronized Start (ARM + AUTO concurrently)")
        print(" (d) Toggle Debug Mode")
        print("="*60 + "\n")

    def upload_mission(self, filename):
        wp_loader = mavwp.MAVWPLoader()
        try:
            wp_loader.load(filename)
            print(f"\n[MISSION] Loaded {wp_loader.count()} waypoints from {filename}")
        except Exception as e:
            print(f"\n[MISSION ERR] Failed to load {filename}: {e}")
            return
        
        self.uploading_mission = True
        time.sleep(1.5) 
        
        for name, conn in [("SITL 1", self.mav_conn1), ("SITL 2", self.mav_conn2)]:
            print(f"[MISSION] Clearing old waypoints on {name}...")
            conn.waypoint_clear_all_send()
            conn.recv_match(type=['MISSION_ACK'], blocking=True, timeout=2)
            
            print(f"[MISSION] Uploading {wp_loader.count()} waypoints to {name}...")
            conn.waypoint_count_send(wp_loader.count())
            
            for i in range(wp_loader.count()):
                msg = conn.recv_match(type=['MISSION_REQUEST'], blocking=True, timeout=2)
                if not msg:
                    print(f"[MISSION ERR] Timeout waiting for MISSION_REQUEST from {name}")
                    break
                conn.mav.send(wp_loader.wp(msg.seq))
            
            ack = conn.recv_match(type=['MISSION_ACK'], blocking=True, timeout=2)
            if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print(f"[MISSION] Successfully uploaded to {name}!")
            else:
                print(f"[MISSION ERR] Failed to verify mission acceptance on {name}.")
                
        self.uploading_mission = False

    def _arm_and_set_auto(self, conn):
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0
        )
        conn.mav.set_mode_send(
            conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            3
        )

    def trigger_concurrent_start(self):
        print("\n[SYNC] >>> EXECUTING CONCURRENT ARM & AUTO TAKEOFF <<<")
        with ThreadPoolExecutor(max_workers=2) as executor:
            futures = [
                executor.submit(self._arm_and_set_auto, self.mav_conn1),
                executor.submit(self._arm_and_set_auto, self.mav_conn2)
            ]
            for future in futures:
                future.result() 
        print("[SYNC] Both instances commanded!\n")

    def thread_mavlink_monitor(self, mav_conn, name):
        print(f"[{name}] Waiting for first heartbeat...")
        mav_conn.wait_heartbeat()
        print(f"[{name}] Heartbeat received!")
        
        last_status = None
        while self.running:
            if self.uploading_mission:
                time.sleep(0.5)
                continue
                
            msg = mav_conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) > 0
                if is_armed != last_status:
                    last_status = is_armed
                    state_str = "ARMED" if is_armed else "DISARMED"
                    print(f"\n[MAVLINK] >>> {name} is now {state_str} <<<\n")

    def stop(self):
        self.running = False
        if self.ser_read: self.ser_read.close()
        if self.ser_write: self.ser_write.close()
        self.sock_gazebo_in.close()
        self.sock_sitl1_in.close()
        self.sock_sitl2_in.close()
        self.sock_out.close()
        self.mav_conn1.close()
        self.mav_conn2.close()
        print("Bridge stopped.")


if __name__ == "__main__":
    bridge = DualSITLHardwareBridge()
    bridge.start()
    try:
        while True:
            cmd = input(">> ").strip().lower()
            if cmd == 'm':
                filename = input("Enter mission filename (e.g., mission.txt): ").strip()
                bridge.upload_mission(filename)
            elif cmd == 's':
                bridge.trigger_concurrent_start()
            elif cmd == 'd':
                bridge.debug_mode = not bridge.debug_mode
                state = "ON" if bridge.debug_mode else "OFF"
                print(f">>> Debug prints turned {state}")
            elif not cmd:
                pass
            else:
                bridge.print_menu()
    except KeyboardInterrupt:
        print("\nInterrupt received. Closing connections...")
        bridge.stop()
        sys.exit(0)