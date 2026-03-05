import time
import socket
import threading
import sys
from pymavlink import mavutil, mavwp
from concurrent.futures import ThreadPoolExecutor

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
        
        try:
            print("Setting up FDM Physics FDM FDM routing FDM...")
            # FDM FDM FDM Socket setup FDM FDM FDM FDM FDM
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
            # REMOVED wait_heartbeat() from here to prevent the deadlock!
        except Exception as e:
            print(f"MAVLink setup failed: {e}")
            sys.exit(1)

    def start(self):
        self.running = True
        # Start FDM physics threads IMMEDIATELY so SITL can boot
        threading.Thread(target=self.thread_gazebo_to_sitls, daemon=True).start()
        threading.Thread(target=self.thread_sitl_handler, args=(self.sock_sitl1_in, True), daemon=True).start()
        threading.Thread(target=self.thread_sitl_handler, args=(self.sock_sitl2_in, False), daemon=True).start()
        
        # Start MAVLink monitors (they will passively wait for SITL to finish booting)
        threading.Thread(target=self.thread_mavlink_monitor, args=(self.mav_conn1, "SITL 1"), daemon=True).start()
        threading.Thread(target=self.thread_mavlink_monitor, args=(self.mav_conn2, "SITL 2"), daemon=True).start()

        self.print_menu()

    def print_menu(self):
        print("\n" + "="*60)
        print("DUAL SITL BRIDGE: ACTIVE REPLICATION MODE")
        print("="*60)
        print(" (1) Use SITL 1 Command Output")
        print(" (2) Use SITL 2 Command Output")
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
        mav_conn.wait_heartbeat() # It safely blocks HERE now, inside the thread!
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

    def thread_gazebo_to_sitls(self):
        while self.running:
            try:
                data, addr = self.sock_gazebo_in.recvfrom(2048)
                if self.debug_mode:
                    print(f"[GAZEBO IN] {len(data)} bytes from {addr}")
                self.sock_out.sendto(data, SITL1_PHYSICS_IN)
                self.sock_out.sendto(data, SITL2_PHYSICS_IN)
            except Exception as e:
                if self.running: print(f"Gazebo In Error: {e}")

    def thread_sitl_handler(self, sock, is_primary):
        sitl_name = "SITL 1" if is_primary else "SITL 2"
        while self.running:
            try:
                data, addr = sock.recvfrom(1024)
                if self.debug_mode:
                    print(f"[{sitl_name} IN] {len(data)} bytes from {addr}")
                
                if self.use_primary_sitl == is_primary:
                    self.sock_out.sendto(data, (GAZEBO_TARGET_IP, GAZEBO_TARGET_PORT))
            except socket.timeout:
                continue
            except Exception as e:
                if self.running: print(f"SITL Handler Error: {e}")

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
                bridge.upload_mission(filename)
            elif cmd == 's':
                bridge.trigger_concurrent_start()
            elif cmd == 'd':
                bridge.debug_mode = not bridge.debug_mode
                state = "ON" if bridge.debug_mode else "OFF"
                print(f">>> Debug prints turned {state}")
            else:
                bridge.print_menu()
    except KeyboardInterrupt:
        bridge.stop()