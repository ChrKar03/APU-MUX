import time
import socket
import threading
import sys
import logging as lg
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
GAZEBO_LISTEN_PORT = 9002  
GAZEBO_TARGET_IP = "127.0.0.1"
GAZEBO_TARGET_PORT = 9003  

SITL1_LISTEN_PORT = 9012   
SITL1_PHYSICS_IN = ("127.0.0.1", 9013) 

SITL2_LISTEN_PORT = 9022   
SITL2_PHYSICS_IN = ("127.0.0.1", 9023) 

MAVLINK_SOURCE1 = "udpin:127.0.0.1:14550"
MAVLINK_SOURCE2 = "udpin:127.0.0.1:14560"

hz = 0.0
avg_latency_ms = 0.0

class DualSITLBridge:
    def __init__(self):
        self.running = False
        self.use_primary_sitl = True 
        self.debug_mode = False  
        self.uploading_mission = False
        
        self.log_sitl1 = lg.getLogger("SITL_1")
        self.log_sitl2 = lg.getLogger("SITL_2")

        try:
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
        
        threading.Thread(target=self.thread_lockstep_physics, daemon=True).start()
        threading.Thread(target=self.thread_mavlink_monitor, args=(self.mav_conn1, "SITL 1", 1), daemon=True).start()
        threading.Thread(target=self.thread_mavlink_monitor, args=(self.mav_conn2, "SITL 2", 2), daemon=True).start()

        self.print_menu()

    def print_menu(self):
        print("\n" + "="*60)
        print("DUAL SITL BRIDGE: ACTIVE REPLICATION MODE")
        print("="*60)
        print(" (1) Use SITL 1 Command Output")
        print(" (2) Use SITL 2 Command Output")
        print(" (m) Load and Upload Mission File (.txt)")
        print(" (s) Synchronized Start (Instantaneous Lockstep Takeoff)")
        print(" (n) Remove SITL Noise (Zero out SIM parameters)")
        print(" (d) Toggle Debug Mode")
        print("="*60 + "\n")

    def thread_lockstep_physics(self):
        global hz, avg_latency_ms
        print("[PHYSICS] Starting strict Lockstep FDM loop...")
        
        self.sock_sitl1_in.settimeout(None)
        self.sock_sitl2_in.settimeout(None)
        self.sock_gazebo_in.settimeout(None)

        frame_count = 0
        sw_latency_sum = 0.0
        last_report_time = time.perf_counter()
        # ---------------------------------------

        while self.running:
            try:
                # 1. BARRIER: Wait for SITL 1's physics frame
                pkt1, addr1 = self.sock_sitl1_in.recvfrom(4096)
                
                # 2. BARRIER: Wait for SITL 2's physics frame
                pkt2, addr2 = self.sock_sitl2_in.recvfrom(4096)

                # 3. Log the data for CSV analysis
                self.log_sitl1.debug(f"{pkt1.hex()}")
                self.log_sitl2.debug(f"{pkt2.hex()}")

                if self.debug_mode:
                    print(f"[FDM] Synced Frame - Forwarding to Gazebo")

                # 4. Forward active SITL's packet to Gazebo
                active_pkt = pkt1 if self.use_primary_sitl else pkt2
                
                # --- START PROPAGATION TIMER ---
                sw_start_time = time.perf_counter()
                
                self.sock_out.sendto(active_pkt, (GAZEBO_TARGET_IP, GAZEBO_TARGET_PORT))

                # 5. BARRIER: Wait for Gazebo to compute physics and reply
                gazebo_data, g_addr = self.sock_gazebo_in.recvfrom(65000)
                
                # --- STOP PROPAGATION TIMER ---
                sw_end_time = time.perf_counter()
                sw_latency_sum += (sw_end_time - sw_start_time)

                # 6. Send exact Gazebo sensor data back to both SITLs back-to-back
                self.sock_out.sendto(gazebo_data, SITL1_PHYSICS_IN)
                self.sock_out.sendto(gazebo_data, SITL2_PHYSICS_IN)

                frame_count += 1
                current_time = time.perf_counter()
                elapsed_since_report = current_time - last_report_time
                
                if elapsed_since_report >= 1.0:
                    hz = frame_count / elapsed_since_report
                    avg_latency_ms = (sw_latency_sum / frame_count) * 1000

                    frame_count = 0
                    sw_latency_sum = 0.0
                    last_report_time = current_time
                # ----------------------------------------------------

            except Exception as e:
                if self.running: lg.error(f"Lockstep Physics Error: {e}")

    def trigger_instant_start(self):
        print("\n[SYNC] >>> PREPARING SYNCHRONIZED START <<<")

        def wait_for_gps_and_prep(conn):
            # 1. Wait for GPS
            print(f"[SYNC] System {conn.target_system} waiting for 3D GPS lock...")
            while True:
                msg = conn.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2.0)
                if msg and msg.get_srcSystem() == conn.target_system and msg.fix_type >= 3:
                    print(f"[SYNC] System {conn.target_system} achieved 3D GPS Lock!")
                    break

            # 2. Set AUTO_OPTIONS to 3 (Allows arming directly into AUTO mode)
            print(f"[SYNC] Setting AUTO_OPTIONS=3 on System {conn.target_system}...")
            conn.mav.param_set_send(
                conn.target_system,
                conn.target_component,
                b'AUTO_OPTIONS',
                3.0,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            time.sleep(0.5)

        # PHASE 1: Pre-launch prep
        with ThreadPoolExecutor(max_workers=2) as executor:
            futures = [
                executor.submit(wait_for_gps_and_prep, self.mav_conn1),
                executor.submit(wait_for_gps_and_prep, self.mav_conn2)
            ]
            for future in futures:
                future.result() 

        print("\n[SYNC] Both systems prepared. Packing instantaneous launch payloads...")
        time.sleep(1.0) 

        # PHASE 2: Pre-pack binary payloads to bypass Python GIL overhead
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
                if msg.get_srcSystem() != expected_sysid:
                    continue

                is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) > 0
                if is_armed != last_status:
                    last_status = is_armed
                    state_str = "ARMED" if is_armed else "DISARMED"
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
        
        while conn.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=False):
            pass
        
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

            if msg.get_srcSystem() != conn.target_system:
                continue

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

        if retries >= MAX_RETRIES:
            print(f"[MISSION ERR] Aborting upload to {name} after maximum timeouts.")

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
            'SIM_GYR1_RND', 'SIM_GYR2_RND', 'SIM_GYR3_RND',
            'SIM_ACC1_RND', 'SIM_ACC2_RND', 'SIM_ACC3_RND',
            'SIM_BARO_RND', 'SIM_MAG1_RND', 'SIM_MAG2_RND',
            'SIM_GPS_NOISE', 'SIM_WIND_SPD', 'SIM_WIND_TURB'
        ]
        
        for param in params_to_zero:
            conn.mav.param_set_send(
                conn.target_system, conn.target_component, param.encode('utf-8'), 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
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
            if cmd == '1':
                bridge.use_primary_sitl = True
                print(">>> Switched active outputs to SITL 1")
            elif cmd == '2':
                bridge.use_primary_sitl = False
                print(">>> Switched active outputs to SITL 2")
            elif cmd == 'm':
                filename = input("Enter mission filename (e.g., mission.txt): ").strip()
                with ThreadPoolExecutor(max_workers=2) as executor:
                    futures = [
                        executor.submit(bridge.upload_mission, bridge.mav_conn1, filename, "SITL 1"),
                        executor.submit(bridge.upload_mission, bridge.mav_conn2, filename, "SITL 2")
                    ]
                    for future in futures:
                        future.result() 
                print("[SYNC] Both instances successfully got the mission!\n")
            elif cmd == 's':
                bridge.trigger_instant_start()
            elif cmd == 'n':
                with ThreadPoolExecutor(max_workers=2) as executor:
                    futures = [
                        executor.submit(bridge.remove_noise, bridge.mav_conn1, "SITL 1"),
                        executor.submit(bridge.remove_noise, bridge.mav_conn2, "SITL 2")
                    ]
                    for future in futures:
                        future.result() 
                print("[CONFIG] Noise removed from both instances!\n")
            elif cmd == 'd':
                bridge.debug_mode = not bridge.debug_mode
                state = "ON" if bridge.debug_mode else "OFF"
                print(f">>> Debug prints turned {state}")
            else:
                bridge.print_menu()
    except KeyboardInterrupt:
        bridge.stop()
    print(f"[METRICS] FDM Loop: {hz:.1f} Hz | Gazebo Propagation RTT: {avg_latency_ms:.2f} ms")