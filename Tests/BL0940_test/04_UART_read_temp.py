import serial
import time

def read_register(ser, addr):
    frame = bytes([0x58, addr])
    print(f"Sent read frame: {[hex(b) for b in frame]}")
    ser.reset_input_buffer()
    ser.write(frame)
    time.sleep(0.1)
    
    if ser.in_waiting >= 4:
        response = ser.read(4)
        data_l, data_m, data_h, checksum = response
        checksum_calc = (((0x58 + addr + data_l + data_m + data_h) & 0xFF) ^ 0xFF)
        if checksum != checksum_calc:
            print(f"Checksum mismatch! Got 0x{checksum:02X}, expected 0x{checksum_calc:02X}")
            return None
        return data_l | (data_m << 8) | (data_h << 16)
    else:
        print("No response or incomplete data.")
        return None

def get_temperature(raw_data):
    # Convert to signed 16-bit after shifting left and back
    raw_temp = ((raw_data << 6) & 0xFFFFFF) >> 6  # ensure 24-bit then align bits
    if raw_temp & (1 << 15):  # negative number check
        raw_temp -= 1 << 16
    return (170.0 / 448.0) * (raw_temp / 2.0 - 32.0) - 45

# ---------- MAIN ----------
COM_PORT = 'COM3'

try:
    ser = serial.Serial(
        port=COM_PORT,
        baudrate=4800,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.2
    )

    if ser.is_open:
        print(f"Opened {COM_PORT}")
        while True:
            raw = read_register(ser, 0x0E)
            if raw is not None:
                temp = get_temperature(raw)
                print(f"Temperature: {temp:.2f} °C")
            else:
                print("Failed to read temperature.")
            time.sleep(1)

except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
