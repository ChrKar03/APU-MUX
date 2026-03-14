import sys
import time
import socket
import struct
import threading
import logging as lg
from pymavlink import mavutil, mavwp
from concurrent.futures import ThreadPoolExecutor

# ---------------------------------------------------------
# LOGGER CONFIGURATION (SPLIT CONSOLE AND FILE)
# ---------------------------------------------------------
# 1. File Handler: Saves raw data to a CSV for analysis
file_handler = lg.FileHandler('sitl_analysis.csv', mode='w')
# Format as: Timestamp(ms), LoggerName(SITL), HexData
file_handler.setFormatter(lg.Formatter('%(asctime)s.%(msecs)03d,%(name)s,%(message)s', datefmt='%H:%M:%S'))
file_handler.setLevel(lg.DEBUG)

# 2. Console Handler: Prints clean info to your terminal
console_handler = lg.StreamHandler()
console_handler.setFormatter(lg.Formatter('%(asctime)s [%(levelname)s] %(message)s', datefmt='%H:%M:%S'))
console_handler.setLevel(lg.INFO) # Ignores DEBUG messages so your terminal doesn't flood

lg.basicConfig(
    level=lg.DEBUG,
    handlers=[file_handler, console_handler]
)

# ---------------------------------------------------------
# CORRECTED PORT MAPPINGS
# ---------------------------------------------------------

# Gazebo FDM Ports
GAZEBO_LISTEN_PORT = 9002  # Bridge receives FDM FDM FDM FDM FDM FROM Gazebo FDM here
GAZEBO_TARGET_IP = "127.0.0.1"
GAZEBO_TARGET_PORT = 9003  # Bridge sends PWM commands TO Gazebo FDM FDM here

# SITL 1 FDM Ports
SITL1_LISTEN_PORT = 9012   # Bridge receives FDM PWM FDM FROM SITL 1 FDM here
SITL1_PHYSICS_IN = ("127.0.0.1", 9013) # Bridge sends FDM FDM FDM FDM TO SITL 1 FDM here

# SITL 2 FDM Ports
SITL2_LISTEN_PORT = 9022   # Bridge receives FDM PWM FDM FROM SITL 2 here
SITL2_PHYSICS_IN = ("127.0.0.1", 9023) # Bridge sends FDM FDM TO SITL 2 here

MAVLINK_SOURCE1 = "udpin:127.0.0.1:14550"
MAVLINK_SOURCE2 = "udpin:127.0.0.1:14560"

class DualSITLBridge:
    def __init__(self):
        self.running = False
        self.use_primary_sitl = True 
        self.debug_mode = False  
        self.uploading_mission = False
        
        # Create dedicated loggers for each SITL to make the CSV clean
        self.log_sitl1 = lg.getLogger("SITL_1")
        self.log_sitl2 = lg.getLogger("SITL_2")

        try:
            print("Setting up FDM Physics FDM FDM routing FDM...")

            # FDM Socket setup
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

        try:
            print("Connecting to MAVLink streams...")
            self.mav_conn1 = mavutil.mavlink_connection(MAVLINK_SOURCE1)
            self.mav_conn2 = mavutil.mavlink_connection(MAVLINK_SOURCE2)
        except Exception as e:
            print(f"MAVLink setup failed: {e}")
            sys.exit(1)

    def start(self):
        self.running = True
        # Start FDM physics threads IMMEDIATELY so SITL can boot
        # threading.Thread(target=self.thread_gazebo_to_sitls, daemon=True).start()
        # threading.Thread(target=self.thread_sitl_handler, args=(self.sock_sitl1_in, True), daemon=True).start()
        # threading.Thread(target=self.thread_sitl_handler, args=(self.sock_sitl2_in, False), daemon=True).start()
        
        # Lockstep Simulation
        threading.Thread(target=self.thread_lockstep_physics, daemon=True).start()

        # Pass the expected System ID (1 and 2) to the monitor threads
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
        print(" (s) Synchronized Start (ARM + AUTO concurrently)")
        print(" (n) Remove SITL Noise (Zero out SIM parameters)") # <-- Added this
        print(" (d) Toggle Debug Mode")
        print("="*60 + "\n")

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
        
        # 1. Flush out any stale messages in the buffer
        while conn.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=False):
            pass
        
        # 2. Clear existing mission
        print(f"[{name}] Clearing old waypoints...")
        conn.mav.mission_clear_all_send(
            conn.target_system, 
            conn.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )

        # Wait for clear ACK
        clear_ack = None
        start_time = time.time()
        while time.time() - start_time < 3.0:
            msg = conn.recv_match(type=['MISSION_ACK'], blocking=True, timeout=0.5)
            if msg and msg.get_srcSystem() == conn.target_system:
                clear_ack = msg
                break
        
        if not clear_ack or clear_ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print(f"[MISSION ERR] Failed to clear mission on {name}. Proceeding anyway...")

        # 3. Initiate the upload by sending the MISSION_COUNT
        print(f"[{name}] Sending MISSION_COUNT ({wp_loader.count()})...")
        conn.mav.mission_count_send(
            conn.target_system, 
            conn.target_component, 
            wp_loader.count(), 
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )

        # 4. State Machine Loop for Upload
        upload_complete = False
        last_msg_sent = 'COUNT'  # Tracks whether to resend COUNT or a specific WAYPOINT
        last_seq_requested = 0
        retries = 0
        MAX_RETRIES = 5

        while not upload_complete and retries < MAX_RETRIES:
            # The MAVLink protocol specifies a timeout should be used.
            # We use a slightly longer timeout (3 seconds) to account for ArduPilot's EKF delay on WP 0
            msg = conn.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=1.5)

            # TIMEOUT HANDLING: Resend the last message if the drone goes quiet
            if not msg:
                retries += 1
                print(f"[{name}] Timeout waiting for drone. Retrying... ({retries}/{MAX_RETRIES})")
                if last_msg_sent == 'COUNT':
                    conn.mav.mission_count_send(
                        conn.target_system, conn.target_component, 
                        wp_loader.count(), mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                    )
                elif last_msg_sent == 'ITEM':
                    # Re-send the last requested waypoint
                    wp = wp_loader.wp(last_seq_requested)
                    self._send_mission_item_int(conn, wp, last_seq_requested)
                continue

            # CRITICAL: Ignore messages from other Ground Stations (e.g., MAVProxy on SysID 255)
            if msg.get_srcSystem() != conn.target_system:
                continue

            # Reset retries since we got a valid response from the drone
            retries = 0 

            # HANDLE MISSION_ACK (Upload Finished or Errored)
            if msg.get_type() == 'MISSION_ACK':
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    print(f"[MISSION] >>> Successfully uploaded all {wp_loader.count()} waypoints to {name}! <<<")
                    upload_complete = True
                else:
                    print(f"[MISSION ERR] Mission rejected by {name}. MAV_MISSION_RESULT Error Code: {msg.type}")
                    break

            # HANDLE MISSION_REQUEST_INT (Drone asking for a specific waypoint)
            elif msg.get_type() in ['MISSION_REQUEST', 'MISSION_REQUEST_INT']:
                # The drone dictates the sequence. We must supply exactly the seq it asks for.
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

    # --- Helper Method to keep the main logic clean ---
    def _send_mission_item_int(self, conn, wp, seq):
        """Formats and sends a single waypoint as a MISSION_ITEM_INT"""
        conn.mav.mission_item_int_send(
            conn.target_system,
            conn.target_component,
            seq,
            wp.frame,
            wp.command,
            wp.current,
            wp.autocontinue,
            wp.param1,
            wp.param2,
            wp.param3,
            wp.param4,
            int(wp.x * 10**7),  # Scaled integer for Lat
            int(wp.y * 10**7),  # Scaled integer for Lon
            wp.z,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )

    # First way: GUIDED, ARM, TAKEOFF, AUTO
    # def _arm_and_set_auto(self, conn):
    #     print(f"[SYNC] System {conn.target_system} waiting for 3D GPS lock...")

    #     # 1. Loop until we get a solid GPS 3D fix
    #     while True:
    #         msg = conn.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2.0)
    #         if msg and msg.get_srcSystem() == conn.target_system:
    #             if msg.fix_type >= 3:
    #                 print(f"[SYNC] System {conn.target_system} achieved 3D GPS Lock!")
    #                 break

    #     # 2. Change mode to GUIDED
    #     print(f"[SYNC] Commanding GUIDED mode on System {conn.target_system}...")
    #     conn.mav.set_mode_send(
    #         conn.target_system,
    #         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    #         4  # 4 = GUIDED
    #     )
    #     time.sleep(1.0)

    #     # 3. The Bulletproof Arming Loop
    #     print(f"[SYNC] Attempting to ARM System {conn.target_system} (Waiting for EKF)...")
    #     armed = False
    #     while not armed:
    #         conn.mav.command_long_send(
    #             conn.target_system, conn.target_component,
    #             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
    #             1, 0, 0, 0, 0, 0, 0
    #         )

    #         ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=2.0)
            
    #         if ack and ack.get_srcSystem() == conn.target_system:
    #             if ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
    #                 if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    #                     print(f"[SYNC] System {conn.target_system} ARMED successfully!")
    #                     armed = True
    #                 else:
    #                     print(f"[SYNC] System {conn.target_system} EKF not ready. Retrying in 2 seconds...")
    #                     time.sleep(2.0)

    #     # Give the motors a split second to spool up at idle
    #     time.sleep(1.0)

    #     # 4. EXPLICIT GUIDED TAKEOFF (The Magic Fix)
    #     print(f"[SYNC] Sending explicit TAKEOFF command to System {conn.target_system}...")
    #     conn.mav.command_long_send(
    #         conn.target_system, conn.target_component,
    #         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
    #         0, 0, 0, 0, 0, 0, 30.0  # Param 7 is Altitude. We command it to lift to 30 meters.
    #     )
        
    #     # Wait 5 seconds to let the drone physically leave the ground and build upward momentum
    #     print(f"[SYNC] System {conn.target_system} lifting off! Waiting 5 seconds...")
    #     time.sleep(5.0)

    #     # 5. Change mode to AUTO to trigger the rest of the mission
    #     print(f"[SYNC] Handing over System {conn.target_system} to AUTO mode...")
    #     conn.mav.set_mode_send(
    #         conn.target_system,
    #         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    #         3  # 3 = AUTO
    #     )

    # Second way: STABILIZE, ARM, AUTO, (Fake Pilot Throttle)
    def _arm_and_set_auto(self, conn):
        print(f"[SYNC] System {conn.target_system} waiting for 3D GPS lock...")

        # 1. Loop until we get a solid GPS 3D fix
        while True:
            msg = conn.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2.0)
            if msg and msg.get_srcSystem() == conn.target_system:
                if msg.fix_type >= 3:
                    print(f"[SYNC] System {conn.target_system} achieved 3D GPS Lock!")
                    break

        # 2. Change mode to a standby mode (STABILIZE = 0, GUIDED = 4) to allow arming
        print(f"[SYNC] Commanding STABILIZE mode on System {conn.target_system}...")
        conn.mav.set_mode_send(
            conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            0  # 0 = STABILIZE
        )
        time.sleep(1.0)

        # 3. The Bulletproof Arming Loop
        print(f"[SYNC] Attempting to ARM System {conn.target_system} (Waiting for EKF)...")
        armed = False
        while not armed:
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0
            )

            ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=2.0)
            if ack and ack.get_srcSystem() == conn.target_system:
                if ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print(f"[SYNC] System {conn.target_system} ARMED successfully!")
                        armed = True
                    else:
                        print(f"[SYNC] System {conn.target_system} EKF not ready. Retrying in 2 seconds...")
                        time.sleep(2.0)

        time.sleep(1.0)

        # 4. Switch to AUTO mode
        print(f"[SYNC] Commanding AUTO mode on System {conn.target_system}...")
        conn.mav.set_mode_send(
            conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            3  # 3 = AUTO
        )
        time.sleep(1.0)

        # 5. FAKE THE HUMAN PILOT: Push the virtual throttle stick up
        print(f"[SYNC] Sending virtual RC Throttle to trigger AUTO takeoff...")
        
        # In MAVLink, 65535 means "Release this channel / do not override". 
        # We only want to override Channel 3 (Throttle) to 1600 PWM.
        conn.mav.rc_channels_override_send(
            conn.target_system,
            conn.target_component,
            65535,  # Ch 1: Roll 
            65535,  # Ch 2: Pitch 
            1600,   # Ch 3: Throttle (1600 PWM = Above Mid-Stick)
            65535,  # Ch 4: Yaw 
            65535,  # Ch 5
            65535,  # Ch 6
            65535,  # Ch 7
            65535   # Ch 8
        )

    # First: Execute Concurent the command threads.
    # def trigger_concurrent_start(self):
    #     print("\n[SYNC] >>> EXECUTING CONCURRENT ARM & AUTO TAKEOFF <<<")
    #     with ThreadPoolExecutor(max_workers=2) as executor:
    #         futures = [
    #             executor.submit(self._arm_and_set_auto, self.mav_conn1),
    #             executor.submit(self._arm_and_set_auto, self.mav_conn2)
    #         ]
    #         for future in futures:
    #             future.result() 
    #     print("[SYNC] Both instances commanded!\n")

    # Second: Pre-pack all the commands and send them secuentially (best results yet).
    # def trigger_concurrent_start(self):
    #     print("\n[SYNC] >>> PREPARING SYNCHRONIZED START <<<")

    #     # --- Phase 1 Helpers (Runs asynchronously) ---
    #     def wait_for_gps_and_arm(conn):
    #         # 1. Wait for GPS
    #         print(f"[SYNC] System {conn.target_system} waiting for 3D GPS lock...")
    #         while True:
    #             msg = conn.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2.0)
    #             if msg and msg.get_srcSystem() == conn.target_system and msg.fix_type >= 3:
    #                 print(f"[SYNC] System {conn.target_system} achieved 3D GPS Lock!")
    #                 break

    #         # 2. Set STABILIZE
    #         print(f"[SYNC] Commanding STABILIZE on System {conn.target_system}...")
    #         conn.mav.set_mode_send(conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)
    #         time.sleep(1.0)

    #         # 3. Arming Loop
    #         print(f"[SYNC] Attempting to ARM System {conn.target_system}...")
    #         armed = False
    #         while not armed:
    #             conn.mav.command_long_send(
    #                 conn.target_system, conn.target_component,
    #                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
    #             )
    #             ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=2.0)
    #             if ack and ack.get_srcSystem() == conn.target_system:
    #                 if ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    #                     print(f"[SYNC] System {conn.target_system} ARMED successfully!")
    #                     armed = True
    #                 else:
    #                     time.sleep(2.0)

    #     # --- EXECUTE PHASE 1 ---
    #     # Run the blocking arming sequences concurrently so we don't wait twice as long
    #     with ThreadPoolExecutor(max_workers=2) as executor:
    #         futures = [
    #             executor.submit(wait_for_gps_and_arm, self.mav_conn1),
    #             executor.submit(wait_for_gps_and_arm, self.mav_conn2)
    #         ]
    #         for future in futures:
    #             future.result() # Wait for both to finish arming

    #     print("\n[SYNC] Both systems ARMED and idling. Preparing instantaneous launch...")
    #     time.sleep(1.0) # Let the idle motors stabilize

    #     # --- Phase 2: PRE-PACK THE LAUNCH COMMANDS ---
    #     print("[SYNC] Pre-packing MAVLink binary payloads...")
        
    #     # Pre-pack AUTO mode commands
    #     auto_msg1 = self.mav_conn1.mav.set_mode_encode(self.mav_conn1.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3).pack(self.mav_conn1.mav)
    #     auto_msg2 = self.mav_conn2.mav.set_mode_encode(self.mav_conn2.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3).pack(self.mav_conn2.mav)

    #     # Pre-pack RC Override commands (Push throttle to 1600)
    #     rc_msg1 = self.mav_conn1.mav.rc_channels_override_encode(
    #         self.mav_conn1.target_system, self.mav_conn1.target_component,
    #         65535, 65535, 1600, 65535, 65535, 65535, 65535, 65535
    #     ).pack(self.mav_conn1.mav)
        
    #     rc_msg2 = self.mav_conn2.mav.rc_channels_override_encode(
    #         self.mav_conn2.target_system, self.mav_conn2.target_component,
    #         65535, 65535, 1600, 65535, 65535, 65535, 65535, 65535
    #     ).pack(self.mav_conn2.mav)


    #     # --- EXECUTE PHASE 2: INSTANTANEOUS BLAST ---
    #     print("[SYNC] >>> EXECUTING INSTANTANEOUS TAKEOFF <<<")
        
    #     # Blast AUTO Mode
    #     self.mav_conn1.write(auto_msg1)
    #     self.mav_conn2.write(auto_msg2)

    #     # Blast RC Override to trigger the climb
    #     self.mav_conn1.write(rc_msg1)
    #     self.mav_conn2.write(rc_msg2)

    #     print("[SYNC] Takeoff commands sent simultaneously!\n")

    #     print("[SYNC] Both instances commanded instantaneously!\n")

    def thread_mavlink_monitor(self, mav_conn, name, expected_sysid):
        print(f"[{name}] Waiting for heartbeat strictly from System ID {expected_sysid}...")

        # 1. Custom wait_heartbeat that ONLY accepts the correct System ID
        while self.running:
            msg = mav_conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            print(f"[{name}] Received heartbeat from System ID {msg.get_srcSystem()}")
            if msg and msg.get_srcSystem() == expected_sysid:
                # Lock this connection to this specific drone
                mav_conn.target_system = expected_sysid
                mav_conn.target_component = msg.get_srcComponent()
                print(f"[{name}] Heartbeat received! Locked to System ID {expected_sysid}.")
                break

        # 2. Normal monitoring loop, but ignoring cross-talk
        last_status = None
        while self.running:
            if self.uploading_mission:
                time.sleep(0.5)
                continue

            msg = mav_conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                # CRITICAL: Ignore heartbeats from the other drone bleeding over the network
                if msg.get_srcSystem() != expected_sysid:
                    continue

                is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) > 0
                if is_armed != last_status:
                    last_status = is_armed
                    state_str = "ARMED" if is_armed else "DISARMED"
                    print(f"\n[MAVLINK] >>> {name} (SysID {expected_sysid}) is now {state_str} <<<\n")

    # First way: Two separate threads with timeouts (simpler, but can have slight desyncs
    def thread_gazebo_to_sitls(self):
        while self.running:
            try:
                data, addr = self.sock_gazebo_in.recvfrom(4096)
                if self.debug_mode:
                    print(f"[GAZEBO IN] {len(data)} bytes from {addr}")
                self.sock_out.sendto(data, SITL1_PHYSICS_IN)
                self.sock_out.sendto(data, SITL2_PHYSICS_IN)
            except Exception as e:
                if self.running: print(f"Gazebo In Error: {e}")

    # Second way: A strict lockstep loop that waits for both SITLs before forwarding to Gazebo (more complex, but perfectly in sync)
    def thread_lockstep_physics(self):
        print("[PHYSICS] Starting strict Lockstep FDM loop...")
        
        # Infinite timeout ensures absolute 1:1 synchronization barriers
        self.sock_sitl1_in.settimeout(None)
        self.sock_sitl2_in.settimeout(None)
        self.sock_gazebo_in.settimeout(None)

        while self.running:
            try:
                # 1. BARRIER: Wait for SITL 1's physics frame
                pkt1, addr1 = self.sock_sitl1_in.recvfrom(4096)
                
                # 2. BARRIER: Wait for SITL 2's physics frame
                pkt2, addr2 = self.sock_sitl2_in.recvfrom(4096)

                # 3. Log the data for CSV analysis
                self.log_sitl1.debug(f"{pkt1.hex()}")
                self.log_sitl2.debug(f"{pkt2.hex()}")

                # 4. Forward active SITL's packet to Gazebo
                active_pkt = pkt1 if self.use_primary_sitl else pkt2
                self.sock_out.sendto(active_pkt, (GAZEBO_TARGET_IP, GAZEBO_TARGET_PORT))

                # 5. BARRIER: Wait for Gazebo to compute physics and reply
                gazebo_data, g_addr = self.sock_gazebo_in.recvfrom(65000)

                # 6. Send exact Gazebo sensor data back to both SITLs back-to-back
                self.sock_out.sendto(gazebo_data, SITL1_PHYSICS_IN)
                self.sock_out.sendto(gazebo_data, SITL2_PHYSICS_IN)

            except Exception as e:
                if self.running: lg.error(f"Lockstep Physics Error: {e}")

    # --- INSTANTANEOUS LAUNCH SEQUENCE ---
    def trigger_instant_start(self):
        print("\n[SYNC] >>> PREPARING SYNCHRONIZED START <<<")

        def wait_for_gps_and_arm(conn):
            print(f"[SYNC] System {conn.target_system} waiting for 3D GPS lock...")
            while True:
                msg = conn.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2.0)
                if msg and msg.get_srcSystem() == conn.target_system and msg.fix_type >= 3:
                    print(f"[SYNC] System {conn.target_system} achieved 3D GPS Lock!")
                    break

            print(f"[SYNC] Commanding STABILIZE on System {conn.target_system}...")
            conn.mav.set_mode_send(conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)
            time.sleep(1.0)

            print(f"[SYNC] Attempting to ARM System {conn.target_system}...")
            armed = False
            while not armed:
                conn.mav.command_long_send(
                    conn.target_system, conn.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
                )
                ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=2.0)
                if ack and ack.get_srcSystem() == conn.target_system:
                    if ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print(f"[SYNC] System {conn.target_system} ARMED successfully!")
                        armed = True
                    else:
                        time.sleep(2.0)

        # PHASE 1: Pre-launch prep (Threaded is fine here because they are anchored to the ground)
        with ThreadPoolExecutor(max_workers=2) as executor:
            futures = [
                executor.submit(wait_for_gps_and_arm, self.mav_conn1),
                executor.submit(wait_for_gps_and_arm, self.mav_conn2)
            ]
            for future in futures:
                future.result() 

        print("\n[SYNC] Both systems ARMED and idling. Preparing instantaneous launch...")
        time.sleep(1.0) 

        # PHASE 2: Pre-pack binary payloads to bypass Python GIL overhead
        print("[SYNC] Pre-packing MAVLink binary payloads...")
        auto_msg1 = self.mav_conn1.mav.set_mode_encode(self.mav_conn1.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3).pack(self.mav_conn1.mav)
        auto_msg2 = self.mav_conn2.mav.set_mode_encode(self.mav_conn2.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3).pack(self.mav_conn2.mav)

        rc_msg1 = self.mav_conn1.mav.rc_channels_override_encode(
            self.mav_conn1.target_system, self.mav_conn1.target_component, 65535, 65535, 1600, 65535, 65535, 65535, 65535, 65535
        ).pack(self.mav_conn1.mav)
        rc_msg2 = self.mav_conn2.mav.rc_channels_override_encode(
            self.mav_conn2.target_system, self.mav_conn2.target_component, 65535, 65535, 1600, 65535, 65535, 65535, 65535, 65535
        ).pack(self.mav_conn2.mav)

        # PHASE 3: INSTANTANEOUS RAW SOCKET BLAST
        print("[SYNC] >>> EXECUTING INSTANTANEOUS TAKEOFF <<<")
        self.mav_conn1.write(auto_msg1)
        self.mav_conn2.write(auto_msg2)
        self.mav_conn1.write(rc_msg1)
        self.mav_conn2.write(rc_msg2)
        print("[SYNC] Takeoff commands sent simultaneously!\n")

    def thread_sitl_handler(self, sock, is_primary):
        sitl_name = "SITL 1" if is_primary else "SITL 2"
        logger = self.log_sitl1 if is_primary else self.log_sitl2

        while self.running:
            try:
                data, addr = sock.recvfrom(4096)

                # ALWAYS log incoming data to the CSV file for analysis (Timestamp, SITL, Hex)
                logger.debug(f"{data.hex()}")

                if self.debug_mode:
                    # Print to console only if debug mode is toggled ON
                    lg.info(f"[{sitl_name} IN] {len(data)} bytes from {addr}")

                # Only forward the active SITL's physics to Gazebo
                if self.use_primary_sitl == is_primary:
                    self.sock_out.sendto(data, (GAZEBO_TARGET_IP, GAZEBO_TARGET_PORT))

            except socket.timeout:
                continue
            except Exception as e:
                if self.running: lg.error(f"SITL Handler Error: {e}")

    def remove_noise(self, conn, name):
        """Sends MAVLink commands to zero out all SITL simulated noise and wind."""
        print(f"\n[CONFIG] Removing sensor noise and wind on {name}...")
        
        # List of all the noise parameters to zero out
        params_to_zero = [
            'SIM_GYR1_RND', 'SIM_GYR2_RND', 'SIM_GYR3_RND',
            'SIM_ACC1_RND', 'SIM_ACC2_RND', 'SIM_ACC3_RND',
            'SIM_BARO_RND',
            'SIM_MAG1_RND', 'SIM_MAG2_RND',
            'SIM_GPS_NOISE',
            'SIM_WIND_SPD', 'SIM_WIND_TURB'
        ]
        
        for param in params_to_zero:
            # Pymavlink requires the parameter ID to be a byte string
            param_id = param.encode('utf-8')
            
            # Send the parameter set command
            conn.mav.param_set_send(
                conn.target_system,
                conn.target_component,
                param_id,
                0.0,  # Set the value to 0.0
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            # A tiny sleep ensures we don't overwhelm the MAVLink buffer
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
                # bridge.trigger_concurrent_start()
                bridge.trigger_instant_start()
            elif cmd == 'd':
                bridge.debug_mode = not bridge.debug_mode
                state = "ON" if bridge.debug_mode else "OFF"
                print(f">>> Debug prints turned {state}")
            elif cmd == 'n':
                with ThreadPoolExecutor(max_workers=2) as executor:
                    futures = [
                        executor.submit(bridge.remove_noise, bridge.mav_conn1, "SITL 1"),
                        executor.submit(bridge.remove_noise, bridge.mav_conn2, "SITL 2")
                    ]
                    for future in futures:
                        future.result() 
                print("[CONFIG] Noise removed from both instances!\n")
            else:
                bridge.print_menu()
    except KeyboardInterrupt:
        bridge.stop()