import sys
import time
import socket
import serial
import struct
import threading
import logging as lg
import serial.tools.list_ports
from pymavlink import mavutil, mavwp
from concurrent.futures import ThreadPoolExecutor

# ---------------------------------------------------------
# LOGGER CONFIGURATION (SPLIT CONSOLE AND FILE)
# ---------------------------------------------------------
file_handler = lg.FileHandler('sitl_analysis.csv', mode='w')
file_handler.setFormatter(lg.Formatter('%(asctime)s.%(msecs)03d,%(name)s,%(message)s', datefmt='%H:%M:%S'))
file_handler.setLevel(lg.DEBUG)

console_handler = lg.StreamHandler()
console_handler.setFormatter(lg.Formatter('%(asctime)s [%(levelname)s] %(message)s', datefmt='%H:%M:%S'))
console_handler.setLevel(lg.INFO) 

lg.basicConfig(
    level=lg.DEBUG,
    handlers=[file_handler, console_handler]
)

# ---------------------------------------------------------
# PORT MAPPINGS
# ---------------------------------------------------------
BOARD_MAP = {
    "READER_BOARD": "003E00263433510237363934", # G4 Board (Reads 4x uint16)
    "WRITER_BOARD": "066EFF3733474B3043022227"  # F4 Board (Writes 8x uint16)
}

BAUDRATE = 115200

GAZEBO_LISTEN_PORT = 9002  
GAZEBO_TARGET_IP = "127.0.0.1"
GAZEBO_TARGET_PORT = 9003  

SITL1_LISTEN_PORT = 9012   
SITL1_PHYSICS_IN = ("127.0.0.1", 9013) 

SITL2_LISTEN_PORT = 9022   
SITL2_PHYSICS_IN = ("127.0.0.1", 9023) 

MAVLINK_SOURCE1 = "udpin:127.0.0.1:14550"
MAVLINK_SOURCE2 = "udpin:127.0.0.1:14560"

class DualSITLBridge:
    def __init__(self):
        self.running = False
        self.debug_mode = False  
        self.uploading_mission = False
        
        # State tracker to prevent Hardware-in-the-Loop deadlocks
        self.sitl_armed = {1: False, 2: False}
        
        self.ser_read = None
        self.ser_write = None
        
        # Updated loggers for Serial data instead of raw SITL packets
        self.log_tx = lg.getLogger("SERIAL_TX")
        self.log_rx = lg.getLogger("SERIAL_RX")

        try:
            print("[HW] Scanning for STM32 boards...")
            ports = serial.tools.list_ports.comports()
            
            for p in ports:
                if BOARD_MAP["READER_BOARD"] in (p.serial_number or ""):
                    try:
                        self.ser_read = serial.Serial(p.device, BAUDRATE)
                        print(f" + [READER (G4)] Found at {p.device}")
                        self.ser_read.reset_input_buffer()
                    except Exception as e:
                        print(f" ! [READER ERR] Failed to open {p.device}: {e}")

                elif BOARD_MAP["WRITER_BOARD"] in (p.serial_number or ""):
                    try:
                        self.ser_write = serial.Serial(p.device, BAUDRATE)
                        print(f" + [WRITER (F4)] Found at {p.device}")
                    except Exception as e:
                        print(f" ! [WRITER ERR] Failed to open {p.device}: {e}")
            if not self.ser_read or not self.ser_write:
                print("\n[WARNING] Could not connect to both STM boards. Hardware-in-the-Loop is disabled.")

            print("Setting up FDM Physics routing...")
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
            
        except Exception as e:
            print(f"Socket setup failed: {e}")
            sys.exit(1)

        try:
            print("Connecting to MAVLink streams...")
            self.mav_conn1 = mavutil.mavlink_connection(MAVLINK_SOURCE1)
            self.mav_conn2 = mavutil.mavlink_connection(MAVLINK_SOURCE2)
        except Exception as e:
            print(f"MAVLink setup failed: {e}")
            sys.exit(1)

    def start(self):
        self.running = True
        
        # 1. Start the Strict Lockstep Physics Loop
        threading.Thread(target=self.thread_lockstep_physics, daemon=True).start()

        # 2. Pass the expected System ID (1 and 2) to the monitor threads
        threading.Thread(target=self.thread_mavlink_monitor, args=(self.mav_conn1, "SITL 1", 1), daemon=True).start()
        threading.Thread(target=self.thread_mavlink_monitor, args=(self.mav_conn2, "SITL 2", 2), daemon=True).start()

        self.print_menu()

    def print_menu(self):
        print("\n" + "="*60)
        print("DUAL SITL BRIDGE: ACTIVE REPLICATION MODE")
        print("="*60)
        print(" (m) Load and Upload Mission File (.txt)")
        print(" (s) Synchronized Start (Instantaneous Lockstep Takeoff)")
        print(" (n) Remove SITL Noise (Zero out SIM parameters)")
        print(" (d) Toggle Debug Mode")
        print("="*60 + "\n")

    def thread_lockstep_physics(self):
        print("[PHYSICS] Starting strict Lockstep FDM loop...")
        
        self.sock_sitl1_in.settimeout(None)
        self.sock_sitl2_in.settimeout(None)
        self.sock_gazebo_in.settimeout(None)

        while self.running:
            try:
                # 1. BARRIER: Wait for SITL frames
                pkt1, addr1 = self.sock_sitl1_in.recvfrom(4096)
                pkt2, addr2 = self.sock_sitl2_in.recvfrom(4096)

                if self.debug_mode:
                    print(f"[FDM] Synced Frame - Forwarding")

                # 2. ONLY route through Serial if at least one SITL is ARMED
                if (self.sitl_armed[1] or self.sitl_armed[2]) and self.ser_write and self.ser_read:
                    # print(f"[HW] ARMED state detected. Engaging Hardware-in-the-Loop for this frame.")
                    sitl1_floats = struct.unpack('<4f', pkt1[:16])
                    sitl2_floats = struct.unpack('<4f', pkt2[:16])

                    def map_to_uint16(f_val):
                        f_val = max(-1.0, min(1.0, f_val))
                        return int((f_val + 1.0) * 1000)

                    u1 = [map_to_uint16(f) for f in sitl1_floats]
                    u2 = [map_to_uint16(f) for f in sitl2_floats]

                    serial_payload = struct.pack('<8H', *u1, *u2)

                    try:
                        if not getattr(self, 'was_armed_last_frame', False):
                            self.ser_read.reset_input_buffer()
                            self.was_armed_last_frame = True

                        self.ser_write.write(serial_payload)
                        self.ser_read.read_until(b'\xaa\xbb')
                        echoed_bytes = self.ser_read.read(8)  

                        # Log the Serial Data directly to the CSV
                        self.log_tx.debug(f"{serial_payload.hex()}")
                        self.log_rx.debug(f"{echoed_bytes.hex()}")

                        echoed_ints = struct.unpack('<4H', echoed_bytes)
                        echoed_floats = [(val / 1000.0) - 1.0 for val in echoed_ints]
                        
                        modified_header = struct.pack('<4f', *echoed_floats)
                        # Stitch the hardware-voted floats to the rest of the original packet
                        active_pkt = modified_header + pkt1[16:]
                            
                    except Exception as e:
                        if self.debug_mode: lg.error(f"[HW] Serial Error: {e}")
                else:
                    # Disarmed state: bypass hardware entirely to keep physics loop fast
                    # print(f"[HW] Both SITLs disarmed. Bypassing Hardware-in-the-Loop for this frame.")
                    self.was_armed_last_frame = False
                    active_pkt = pkt1

                # 3. Forward final packet to Gazebo
                self.sock_out.sendto(active_pkt, (GAZEBO_TARGET_IP, GAZEBO_TARGET_PORT))

                # 4. BARRIER: Wait for Gazebo reply
                gazebo_data, g_addr = self.sock_gazebo_in.recvfrom(65000)

                # 5. Send sensor data back to both SITLs
                self.sock_out.sendto(gazebo_data, SITL1_PHYSICS_IN)
                self.sock_out.sendto(gazebo_data, SITL2_PHYSICS_IN)

            except Exception as e:
                if self.running: lg.error(f"Lockstep Physics Error: {e}")

# def thread_lockstep_physics(self):
#         print("[PHYSICS] Starting strict Lockstep FDM loop...")
        
#         self.sock_sitl1_in.settimeout(None)
#         self.sock_sitl2_in.settimeout(None)
#         self.sock_gazebo_in.settimeout(None)

#         # --- METRICS TRACKING INITIALIZATION ---
#         frame_count = 0
#         hw_latency_sum = 0.0
#         hw_latency_count = 0
#         last_report_time = time.perf_counter()
#         # ---------------------------------------

#         while self.running:
#             try:
#                 # 1. BARRIER: Wait for SITL frames
#                 pkt1, addr1 = self.sock_sitl1_in.recvfrom(4096)
#                 pkt2, addr2 = self.sock_sitl2_in.recvfrom(4096)

#                 if self.debug_mode:
#                     print(f"[FDM] Synced Frame - Forwarding")

#                 # 2. ONLY route through Serial if at least one SITL is ARMED
#                 if (self.sitl_armed or self.sitl_armed) and self.ser_write and self.ser_read:
#                     sitl1_floats = struct.unpack('<4f', pkt1[:16])
#                     sitl2_floats = struct.unpack('<4f', pkt2[:16])

#                     def map_to_uint16(f_val):
#                         f_val = max(-1.0, min(1.0, f_val))
#                         return int((f_val + 1.0) * 1000)

#                     u1 = [map_to_uint16(f) for f in sitl1_floats]
#                     u2 = [map_to_uint16(f) for f in sitl2_floats]

#                     serial_payload = struct.pack('<8H', *u1, *u2)

#                     try:
#                         if not getattr(self, 'was_armed_last_frame', False):
#                             self.ser_read.reset_input_buffer()
#                             self.was_armed_last_frame = True

#                         # --- START LATENCY TIMER ---
#                         hw_start_time = time.perf_counter()
                        
#                         self.ser_write.write(serial_payload)
#                         self.ser_read.read_until(b'\xaa\xbb')
#                         echoed_bytes = self.ser_read.read(8)  
                        
#                         # --- STOP LATENCY TIMER ---
#                         hw_end_time = time.perf_counter()
#                         hw_latency_sum += (hw_end_time - hw_start_time)
#                         hw_latency_count += 1

#                         # Log the Serial Data directly to the CSV
#                         self.log_tx.debug(f"{serial_payload.hex()}")
#                         self.log_rx.debug(f"{echoed_bytes.hex()}")

#                         echoed_ints = struct.unpack('<4H', echoed_bytes)
#                         echoed_floats = [(val / 1000.0) - 1.0 for val in echoed_ints]
                        
#                         modified_header = struct.pack('<4f', *echoed_floats)
#                         # Stitch the hardware-voted floats to the rest of the original packet
#                         active_pkt = modified_header + pkt1[16:]
                            
#                     except Exception as e:
#                         if self.debug_mode: lg.error(f"[HW] Serial Error: {e}")
#                         active_pkt = pkt1  # Safe fallback if serial read times out
#                 else:
#                     # Disarmed state: bypass hardware entirely to keep physics loop fast
#                     self.was_armed_last_frame = False
#                     active_pkt = pkt1

#                 # 3. Forward final packet to Gazebo
#                 self.sock_out.sendto(active_pkt, (GAZEBO_TARGET_IP, GAZEBO_TARGET_PORT))

#                 # 4. BARRIER: Wait for Gazebo reply
#                 gazebo_data, g_addr = self.sock_gazebo_in.recvfrom(65000)

#                 # 5. Send sensor data back to both SITLs
#                 self.sock_out.sendto(gazebo_data, SITL1_PHYSICS_IN)
#                 self.sock_out.sendto(gazebo_data, SITL2_PHYSICS_IN)

#                 # --- CALCULATE AND PRINT METRICS (Every 1 second) ---
#                 frame_count += 1
#                 current_time = time.perf_counter()
#                 elapsed_since_report = current_time - last_report_time
                
#                 if elapsed_since_report >= 1.0:
#                     hz = frame_count / elapsed_since_report
                    
#                     if hw_latency_count > 0:
#                         avg_latency_ms = (hw_latency_sum / hw_latency_count) * 1000
#                         sys.stdout.write(f"\r[METRICS] FDM Loop: {hz:.1f} Hz | HW Propagation RTT: {avg_latency_ms:.2f} ms     ")
#                     else:
#                         sys.stdout.write(f"\r[METRICS] FDM Loop: {hz:.1f} Hz | HW Propagation RTT: N/A (Bypassed)      ")
#                     sys.stdout.flush()

#                     # Reset counters for the next second
#                     frame_count = 0
#                     hw_latency_sum = 0.0
#                     hw_latency_count = 0
#                     last_report_time = current_time
#                 # ----------------------------------------------------

#             except Exception as e:
#                 if self.running: lg.error(f"Lockstep Physics Error: {e}")

    def trigger_instant_start(self):
        print("\n[SYNC] >>> PREPARING SYNCHRONIZED START <<<")

        def wait_for_gps_and_prep(conn):
            print(f"[SYNC] System {conn.target_system} waiting for 3D GPS lock...")
            while True:
                msg = conn.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2.0)
                if msg and msg.get_srcSystem() == conn.target_system and msg.fix_type >= 3:
                    print(f"[SYNC] System {conn.target_system} achieved 3D GPS Lock!")
                    break

            print(f"[SYNC] Setting AUTO_OPTIONS=3 on System {conn.target_system}...")
            conn.mav.param_set_send(conn.target_system, conn.target_component, b'AUTO_OPTIONS', 3.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            time.sleep(0.5)

        with ThreadPoolExecutor(max_workers=2) as executor:
            futures = [executor.submit(wait_for_gps_and_prep, self.mav_conn1), executor.submit(wait_for_gps_and_prep, self.mav_conn2)]
            for future in futures: future.result() 

        print("\n[SYNC] Both systems prepared. Packing instantaneous launch payloads...")
        time.sleep(1.0) 

        auto_msg1 = self.mav_conn1.mav.set_mode_encode(self.mav_conn1.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3).pack(self.mav_conn1.mav)
        auto_msg2 = self.mav_conn2.mav.set_mode_encode(self.mav_conn2.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3).pack(self.mav_conn2.mav)
        arm_msg1 = self.mav_conn1.mav.command_long_encode(self.mav_conn1.target_system, self.mav_conn1.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0).pack(self.mav_conn1.mav)
        arm_msg2 = self.mav_conn2.mav.command_long_encode(self.mav_conn2.target_system, self.mav_conn2.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0).pack(self.mav_conn2.mav)

        print("[SYNC] >>> EXECUTING INSTANTANEOUS TAKEOFF <<<")
        self.mav_conn1.write(auto_msg1)
        self.mav_conn2.write(auto_msg2)
        time.sleep(0.1) 
        self.mav_conn1.write(arm_msg1)
        self.mav_conn2.write(arm_msg2)
        print("[SYNC] Perfect Launch Executed!\n")

    def thread_mavlink_monitor(self, mav_conn, name, expected_sysid):
        print(f"[{name}] Waiting for heartbeat strictly from System ID {expected_sysid}...")

        while self.running:
            msg = mav_conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.get_srcSystem() == expected_sysid:
                mav_conn.target_system = expected_sysid
                mav_conn.target_component = msg.get_srcComponent()
                print(f"[{name}] Heartbeat received! Locked to System ID {expected_sysid}.")
                break

        last_status = None
        while self.running:
            if self.uploading_mission:
                time.sleep(0.5)
                continue

            msg = mav_conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                if msg.get_srcSystem() != expected_sysid: continue

                # Check if drone is armed and update our Hardware-in-the-Loop tracking flag
                is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) > 0
                self.sitl_armed[expected_sysid] = is_armed
                
                if is_armed != last_status:
                    last_status = is_armed
                    state_str = "ARMED (Hardware Engaged)" if is_armed else "DISARMED (Hardware Bypassed)"
                    lg.info(f"\n[MAVLINK] >>> {name} (SysID {expected_sysid}) is now {state_str} <<<\n")

    def upload_mission(self, conn, filename, name):
        wp_loader = mavwp.MAVWPLoader()
        try:
            wp_loader.load(filename)
            print(f"\n[MISSION] Loaded {wp_loader.count()} waypoints from {filename}")
        except Exception as e:
            print(f"\n[MISSION ERR] Failed to load {filename}: {e}")
            return
        
        self.uploading_mission = True
        time.sleep(1.5) 
        
        print(f"\n[MISSION] Starting upload to {name}...")
        
        while conn.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=False): pass
        
        print(f"[{name}] Clearing old waypoints...")
        conn.mav.mission_clear_all_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

        clear_ack = None
        start_time = time.time()
        while time.time() - start_time < 3.0:
            msg = conn.recv_match(type=['MISSION_ACK'], blocking=True, timeout=0.5)
            if msg and msg.get_srcSystem() == conn.target_system:
                clear_ack = msg
                break
        
        if not clear_ack or clear_ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print(f"[MISSION ERR] Failed to clear mission on {name}. Proceeding anyway...")

        print(f"[{name}] Sending MISSION_COUNT ({wp_loader.count()})...")
        conn.mav.mission_count_send(conn.target_system, conn.target_component, wp_loader.count(), mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

        upload_complete = False
        last_msg_sent = 'COUNT'
        last_seq_requested = 0
        retries = 0
        MAX_RETRIES = 5

        while not upload_complete and retries < MAX_RETRIES:
            msg = conn.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=1.5)

            if not msg:
                retries += 1
                print(f"[{name}] Timeout waiting for drone. Retrying... ({retries}/{MAX_RETRIES})")
                if last_msg_sent == 'COUNT':
                    conn.mav.mission_count_send(conn.target_system, conn.target_component, wp_loader.count(), mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
                elif last_msg_sent == 'ITEM':
                    wp = wp_loader.wp(last_seq_requested)
                    self._send_mission_item_int(conn, wp, last_seq_requested)
                continue

            if msg.get_srcSystem() != conn.target_system: continue
            retries = 0 

            if msg.get_type() == 'MISSION_ACK':
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    print(f"[MISSION] >>> Successfully uploaded all {wp_loader.count()} waypoints to {name}! <<<")
                    upload_complete = True
                else:
                    print(f"[MISSION ERR] Mission rejected by {name}. MAV_MISSION_RESULT Error Code: {msg.type}")
                    break
            elif msg.get_type() in ['MISSION_REQUEST', 'MISSION_REQUEST_INT']:
                requested_seq = msg.seq
                if requested_seq < wp_loader.count():
                    wp = wp_loader.wp(requested_seq)
                    self._send_mission_item_int(conn, wp, requested_seq)
                    last_msg_sent = 'ITEM'
                    last_seq_requested = requested_seq
                else:
                    print(f"[{name}] Drone requested out-of-bounds waypoint index: {requested_seq}")

        if retries >= MAX_RETRIES: print(f"[MISSION ERR] Aborting upload to {name} after maximum timeouts.")
        self.uploading_mission = False

    def _send_mission_item_int(self, conn, wp, seq):
        conn.mav.mission_item_int_send(
            conn.target_system, conn.target_component, seq, wp.frame, wp.command, wp.current, wp.autocontinue,
            wp.param1, wp.param2, wp.param3, wp.param4,
            int(wp.x * 10**7), int(wp.y * 10**7), wp.z, mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )

    def remove_noise(self, conn, name):
        print(f"\n[CONFIG] Removing sensor noise and wind on {name}...")
        params_to_zero = [
            'SIM_GYR1_RND', 'SIM_GYR2_RND', 'SIM_GYR3_RND', 'SIM_ACC1_RND', 'SIM_ACC2_RND', 'SIM_ACC3_RND',
            'SIM_BARO_RND', 'SIM_MAG1_RND', 'SIM_MAG2_RND', 'SIM_GPS_NOISE', 'SIM_WIND_SPD', 'SIM_WIND_TURB'
        ]
        for param in params_to_zero:
            conn.mav.param_set_send(conn.target_system, conn.target_component, param.encode('utf-8'), 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            time.sleep(0.05)
        print(f"[CONFIG] Noise parameters successfully zeroed on {name}.")

    def stop(self):
        self.running = False
        self.sock_gazebo_in.close()
        self.sock_sitl1_in.close()
        self.sock_sitl2_in.close()
        self.sock_out.close()
        self.mav_conn1.close()
        self.mav_conn2.close()
        print("Bridge stopped.")

if __name__ == "__main__":
    bridge = DualSITLBridge()
    bridge.start()
    try:
        while True:
            cmd = input(">> ").strip().lower()
            if cmd == 'm':
                filename = input("Enter mission filename (e.g., mission.txt): ").strip()
                with ThreadPoolExecutor(max_workers=2) as executor:
                    futures = [
                        executor.submit(bridge.upload_mission, bridge.mav_conn1, filename, "SITL 1"),
                        executor.submit(bridge.upload_mission, bridge.mav_conn2, filename, "SITL 2")
                    ]
                    for future in futures: future.result() 
                print("[SYNC] Both instances successfully got the mission!\n")
            elif cmd == 's': bridge.trigger_instant_start()
            elif cmd == 'n':
                with ThreadPoolExecutor(max_workers=2) as executor:
                    futures = [
                        executor.submit(bridge.remove_noise, bridge.mav_conn1, "SITL 1"),
                        executor.submit(bridge.remove_noise, bridge.mav_conn2, "SITL 2")
                    ]
                    for future in futures: future.result() 
                print("[CONFIG] Noise removed from both instances!\n")
            elif cmd == 'd':
                bridge.debug_mode = not bridge.debug_mode
                print(f">>> Debug prints turned {'ON' if bridge.debug_mode else 'OFF'}")
            else: bridge.print_menu()
    except KeyboardInterrupt:
        bridge.stop()