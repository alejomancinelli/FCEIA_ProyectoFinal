import serial
import time

# --- Serial communication setup ---
COM_PORT = 'COM3'  # Change to match your system

# --- Register read/write functions ---
def write_register(ser, addr, value):
    data_l = value & 0xFF
    data_m = (value >> 8) & 0xFF
    data_h = (value >> 16) & 0xFF
    checksum = ~(0xA8 + addr + data_l + data_m + data_h) & 0xFF
    frame = bytes([0xA8, addr, data_l, data_m, data_h, checksum])

    ser.reset_input_buffer()
    ser.write(frame)
    print("Sent write frame:", [hex(b) for b in frame])
    time.sleep(0.1)

def read_register(ser, addr):
    frame = bytes([0x58, addr])
    ser.reset_input_buffer()
    ser.write(frame)
    print("Sent read frame:", [hex(b) for b in frame])
    time.sleep(0.1)

    if ser.in_waiting >= 4:
        response = ser.read(4)
        data_l, data_m, data_h, checksum = response
        calc_checksum = ~(0x58 + addr + data_l + data_m + data_h) & 0xFF

        if checksum != calc_checksum:
            print(f"Checksum mismatch! Received: {hex(checksum)}, Expected: {hex(calc_checksum)}")
            return None

        raw_value = data_l | (data_m << 8) | (data_h << 16)
        print(f"Register 0x{addr:02X} value: {raw_value} (0x{raw_value:06X})")
        return raw_value
    else:
        print("No response or incomplete data.")
        return None

# --- Voltage conversion function based on your calibration ---
def get_voltage(raw_data):
    Vref = 1.218
    R8_R12_total = 100.0  # kΩ
    R7 = 24.9  # Ω

    voltage = (raw_data * Vref * R8_R12_total) / (79931.0 * R7)  # Convert kΩ to Ω
    return voltage

# --- Main ---
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
        print(f"Opened {COM_PORT}\n")

        while True:
            raw_vrms = read_register(ser, 0x06)  # V_RMS register
            if raw_vrms is not None:
                voltage = get_voltage(raw_vrms)
                print(f"Voltage: {voltage:.2f} V\n")
            else:
                print("Failed to read voltage.\n")

            time.sleep(1)

    else:
        print(f"Failed to open {COM_PORT}")

except serial.SerialException as e:
    print(f"Serial error: {e}")
