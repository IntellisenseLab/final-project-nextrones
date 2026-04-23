import serial
import time
import struct

def make_packet(speed, radius):
    sub = struct.pack('<BBhh', 0x01, 4, speed, radius)
    length = len(sub)
    checksum = length
    for b in sub: checksum ^= b
    return bytes([0xAA, 0x55, length]) + sub + bytes([checksum])

port = "/dev/ttyUSB0"
print(f"Connecting to {port}...")
try:
    s = serial.Serial(port, 115200, timeout=1)
    print("✅ Port Opened!")
    
    # Try to spin for 2 seconds
    print("Sending SPIN command...")
    packet = make_packet(100, 1) # Speed 100, Radius 1 (Pure spin)
    for _ in range(20):
        s.write(packet)
        time.sleep(0.1)
    
    print("Stopping...")
    stop = make_packet(0, 0)
    s.write(stop)
    s.close()
    print("Done. Did it move?")
except Exception as e:
    print(f"❌ Error: {e}")
