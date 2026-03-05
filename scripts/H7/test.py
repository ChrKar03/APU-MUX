from pymavlink import mavutil
m = mavutil.mavlink_connection('udpin:0.0.0.0:14550')  # listen
print("waiting for heartbeat...")
m.wait_heartbeat()
print("received heartbeat from sysid", m.target_system)
while True:
    msg = m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg:
        print("HB", msg)
    else:
        print("no hb for 5s")