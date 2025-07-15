import serial
import time

def request_full_packet(ser):
    frame = bytes([0x58, 0xAA])
    ser.reset_input_buffer()
    ser.write(frame)
    print(f"Sent frame: {[hex(b) for b in frame]}")
    
    time.sleep(0.1)  # 100 ms to wait for 77 ms response
    
    if ser.in_waiting >= 35:
        response = ser.read(35)
        print("Received 35-byte packet:")
        print(" ".join(f"{b:02X}" for b in response))
        return response
    else:
        print(f"Received only {ser.in_waiting} bytes")
        response = ser.read(ser.in_waiting)
        print("Partial data:", " ".join(f"{b:02X}" for b in response))
        return None

# ----------- MAIN -------------
ser = serial.Serial(
    port='COM3',     # Replace with your actual COM port
    baudrate=4800,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.2
)

if ser.is_open:
    request_full_packet(ser)
else:
    print("Failed to open serial port.")

ser.close()