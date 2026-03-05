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
# Where Gazebo is listening for motor commands
UDP_TARGET_IP = "127.0.0.1"
UDP_TARGET_PORT = 9004

# Connection to the PHYSICAL APU (CUAV V5 Nano)
# Use your specific COM port (e.g., 'COM3' for Windows or '/dev/ttyACM0' for Linux) 
# or a MAVProxy UDP bridge address.
MAVLINK_SOURCE = "COM3" 
MAVLINK_BAUD = 115200

SERIAL_BAUDRATE = 115200

class HITLBridge:
    def __init__(self):
        self.running = False
        self.ser = None
        self.stm32_engaged = False  # Controlled by APU Arming status
        
        # Setup Serial (STM32)
        port_name = self.find_stm32_port()
        if not port_name:
            print("Error: STM32 device not found.")
            exit(1)
        try:
            self.ser = serial.Serial(port_name, SERIAL_BAUDRATE, timeout=1)
            print(f"STM32 connected on {port_name}")
        except serial.SerialException as e:
            print(f"Failed to open STM32 serial port: {e}")
            exit(1)

        # Setup UDP Socket (Script -> Gazebo)
        try:
            self.sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"UDP Output configured for {UDP_TARGET_IP}:{UDP_TARGET_PORT}")
        except Exception as e:
            print(f"Network setup failed: {e}")
            exit(1)

        # Setup MAVLink Connection (Physical APU State Monitor)
        try:
            print(f"Connecting to APU MAVLink stream at {MAVLINK_SOURCE}...")
            # If using a direct serial connection to the APU:
            self.mav_conn = mavutil.mavlink_connection(MAVLINK_SOURCE, baud=MAVLINK_BAUD)
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
        
        # Thread 1: MAVLink Monitor
        t_mav = threading.Thread(target=self.thread_mavlink_monitor, daemon=True)
        t_mav.start()

        # Thread 2: Serial Handler (STM32 -> Gazebo)
        t_ser = threading.Thread(target=self.thread_stm32_to_gazebo, daemon=True)
        t_ser.start()

        print("\n" + "="*60)
        print("HITL BRIDGE RUNNING: WAITING FOR APU HEARTBEAT...")
        print("Data Flow: APU (PWM) -> STM32 -> Script -> Gazebo")
        print("="*60 + "\n")

    def stop(self):
        self.running = False
        if self.ser: self.ser.close()
        if self.sock_out: self.sock_out.close()
        print("Bridge stopped.")

    # ---------------------------------------------------------
    # Thread 1: MAVLink Monitor (The Logic Brain)
    # ---------------------------------------------------------
    def thread_mavlink_monitor(self):
        """
        Listens to HEARTBEAT messages from the physical APU.
        If base_mode has MAV_MODE_FLAG_SAFETY_ARMED bit set -> Forward PWM to Gazebo.
        """
        last_status = False 

        while self.running:
            # Wait for a heartbeat
            msg = self.mav_conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            
            if msg:
                # Check if the ARMED bit (128) is set in base_mode
                is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) > 0
                
                if is_armed != last_status:
                    self.stm32_engaged = is_armed
                    last_status = is_armed
                    
                    if is_armed:
                        print("\n[AUTO] >>> PHYSICAL APU ARMED: FORWARDING PWM TO GAZEBO <<<\n")
                    else:
                        print("\n[AUTO] <<< PHYSICAL APU DISARMED: IGNORING PWM <<<\n")

    # ---------------------------------------------------------
    # Thread 2: STM32 -> Gazebo (Actuator Feedback)
    # ---------------------------------------------------------
    def thread_stm32_to_gazebo(self):
        EXPECTED_BYTES = 8 # 4x uint16s from the STM32

        while self.running:
            # Don't send motor commands to Gazebo if APU is disarmed
            if not self.stm32_engaged:
                self.ser.reset_input_buffer() # Clear old PWM data
                time.sleep(0.05)
                continue

            try:
                if self.ser.in_waiting >= EXPECTED_BYTES:
                    serial_data = self.ser.read(EXPECTED_BYTES)
                    
                    if len(serial_data) == EXPECTED_BYTES:
                        # Unpack the 4 raw PWM values (e.g., 1000 - 2000)
                        uint_vals = struct.unpack('<4H', serial_data)
                        
                        float_vals = []
                        for val in uint_vals:
                            # Normalize the logic here based on what Gazebo expects.
                            # Standard PWM is usually 1000us (min) to 2000us (max).
                            # If Gazebo expects 0.0 to 1.0: 
                            # canonical = (float(val) - 1000.0) / 1000.0
                            
                            # Using the math from your original script:
                            canonical = (float(val) / 2500.0) - 1.0
                            
                            # Clamp values to prevent Gazebo physics explosions
                            canonical = max(-1.0, min(1.0, canonical)) 
                            float_vals.append(canonical)

                        # Pad with 12 zeros to make 16 floats total for the Gazebo plugin
                        float_vals.extend([0.0] * 12)

                        # Pack and send
                        udp_packet = struct.pack('<16f', *float_vals)
                        self.sock_out.sendto(udp_packet, (UDP_TARGET_IP, UDP_TARGET_PORT))
                else:
                    time.sleep(0.001)

            except Exception as e:
                if self.running: print(f"Serial/UDP Error: {e}")

if __name__ == "__main__":
    bridge = HITLBridge()
    try:
        bridge.start()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        bridge.stop()