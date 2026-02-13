import time
import socket
import serial
import threading
import struct
import serial.tools.list_ports
import sys

# Import MAVLink for state detection
from pymavlink import mavutil

# ---------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------
UDP_LISTEN_IP = "127.0.0.1"
UDP_LISTEN_PORT = 9002

UDP_TARGET_IP = "127.0.0.1"
UDP_TARGET_PORT = 9004

# Standard SITL MAVLink output port
MAVLINK_SOURCE = "udp:127.0.0.1:14550" 

SERIAL_BAUDRATE = 115200

class STM32Bridge:
    def __init__(self):
        self.running = False
        self.ser = None
        self.stm32_engaged = False  # Controlled by MAVLink Arming status
        
        # Setup Serial
        port_name = self.find_stm32_port()
        if not port_name:
            print("Error: STM32 device not found.")
            exit(1)
        try:
            self.ser = serial.Serial(port_name, SERIAL_BAUDRATE, timeout=1)
            print(f"Serial connected on {port_name}")
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            exit(1)

        # Setup UDP Sockets (Data Loop)
        try:
            self.sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock_in.bind((UDP_LISTEN_IP, UDP_LISTEN_PORT))
            self.sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except Exception as e:
            print(f"Network setup failed: {e}")
            exit(1)

        # Setup MAVLink Connection (State Monitor)
        try:
            print(f"Connecting to MAVLink stream at {MAVLINK_SOURCE}...")
            self.mav_conn = mavutil.mavlink_connection(MAVLINK_SOURCE)
        except Exception as e:
            print(f"MAVLink setup failed: {e}")
            exit(1)

    def find_stm32_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if (port.manufacturer and "STMicroelectronics" in port.manufacturer) or \
               (port.description and "STM32" in port.description):
                return port.device
        return None

    def start(self):
        self.running = True
        
        # Thread 1: MAVLink Monitor (Auto-Switching Logic)
        t_mav = threading.Thread(target=self.thread_mavlink_monitor, daemon=True)
        t_mav.start()

        # Thread 2: UDP Handler
        t_udp = threading.Thread(target=self.thread_udp_handler, daemon=True)
        t_udp.start()
        
        # Thread 3: Serial Handler
        t_ser = threading.Thread(target=self.thread_serial_to_udp, daemon=True)
        t_ser.start()

        print("\n" + "="*60)
        print("BRIDGE RUNNING: WAITING FOR ARDUPILOT HEARTBEAT...")
        print("Protocol: 16x 32-bit floats")
        print("Mode: AUTOMATIC (Switches to STM32 when ARMED)")
        print("="*60 + "\n")

    def stop(self):
        self.running = False
        if self.ser: self.ser.close()
        if self.sock_in: self.sock_in.close()
        if self.sock_out: self.sock_out.close()
        print("Bridge stopped.")

    # ---------------------------------------------------------
    # Thread 1: MAVLink Monitor (The Logic Brain)
    # ---------------------------------------------------------
    def thread_mavlink_monitor(self):
        """
        Listens to HEARTBEAT messages.
        If base_mode has MAV_MODE_FLAG_SAFETY_ARMED bit set -> Engage STM32.
        Else -> Bypass.
        """
        # Flag to debounce/print status only on change
        last_status = False 

        while self.running:
            # Wait for a heartbeat (timeout 1s to allow loop checks)
            msg = self.mav_conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            
            if msg:
                # Check if the ARMED bit (128) is set in base_mode
                is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) > 0
                
                if is_armed != last_status:
                    self.stm32_engaged = is_armed
                    last_status = is_armed
                    
                    if is_armed:
                        print("\n[AUTO] >>> DRONE ARMED: ENGAGING STM32 LOOP <<<\n")
                    else:
                        print("\n[AUTO] <<< DRONE DISARMED: BYPASSING STM32 <<<\n")

    # ---------------------------------------------------------
    # Thread 2: UDP Handler (SITL -> Serial/Gazebo)
    # ---------------------------------------------------------
    def thread_udp_handler(self):
        EXPECTED_BYTES = 64 # 16 floats * 4 bytes

        while self.running:
            try:
                data, _ = self.sock_in.recvfrom(1024)
                
                # --- BYPASS MODE (Disarmed) ---
                if not self.stm32_engaged:
                    self.sock_out.sendto(data, (UDP_TARGET_IP, UDP_TARGET_PORT))
                    continue

                # --- STM32 MODE (Armed) ---
                if len(data) != EXPECTED_BYTES:
                    continue

                # Unpack 16 float32s
                floats = struct.unpack('<16f', data)

                uint_vals = []
                # Map first 4 channels to [0-5000]
                for i in range(4):
                    val = (floats[i] + 1.0) * 2500.0
                    val = max(0.0, min(5000.0, val))
                    uint_vals.append(int(val))

                # Pack 4 uint16s
                serial_packet = struct.pack('<4H', *uint_vals)
                
                # Burst write to ensure STM32 buffer fills
                for _ in range(4): 
                    self.ser.write(serial_packet)
                    time.sleep(0.002)

            except Exception as e:
                if self.running: print(f"UDP Error: {e}")

    # ---------------------------------------------------------
    # Thread 3: Serial Handler (STM32 -> Gazebo)
    # ---------------------------------------------------------
    def thread_serial_to_udp(self):
        EXPECTED_BYTES = 8 

        while self.running:
            # Don't read garbage if we aren't engaged
            if not self.stm32_engaged:
                time.sleep(0.05)
                continue

            try:
                if self.ser.in_waiting >= EXPECTED_BYTES:
                    serial_data = self.ser.read(EXPECTED_BYTES)
                    
                    if len(serial_data) == EXPECTED_BYTES:
                        uint_vals = struct.unpack('<4H', serial_data)
                        
                        float_vals = []
                        for val in uint_vals:
                            canonical = (float(val) / 2500.0) - 1.0
                            float_vals.append(canonical)

                        # Pad with 12 zeros to make 16 floats total
                        float_vals.extend([0.0] * 12)

                        udp_packet = struct.pack('<16f', *float_vals)
                        self.sock_out.sendto(udp_packet, (UDP_TARGET_IP, UDP_TARGET_PORT))
                else:
                    time.sleep(0.001)

            except Exception as e:
                if self.running: print(f"Serial Error: {e}")

if __name__ == "__main__":
    bridge = STM32Bridge()
    try:
        bridge.start()
        # Main thread simply waits
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        bridge.stop()