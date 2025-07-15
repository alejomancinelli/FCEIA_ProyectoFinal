import serial
import time

def read_register(ser, addr):
    # Send the read command
    frame = bytes([0x58, addr])
    print(f"Sent read frame: {[hex(b) for b in frame]}")
    ser.reset_input_buffer()
    ser.write(frame)
    
    # Allow time for the sensor to respond (at least 72 us, but let's use a small sleep time)
    time.sleep(0.1)  # Adjust if needed based on actual response time
    
    # Check if enough data is available
    if ser.in_waiting >= 4:
        response = ser.read(4)
        data_l, data_m, data_h, checksum = response
        
        # Calculate expected checksum
        checksum_calculated = (((0x58 + addr + data_l + data_m + data_h) & 0xFF) ^ 0xFF)
        
        # Verify checksum
        if checksum == checksum_calculated:
            raw_value = data_l | (data_m << 8) | (data_h << 16)
            print(f"Register 0x{addr:02X} value: {raw_value} (0x{raw_value:06X})")
            return raw_value
        else:
            print(f"Checksum mismatch! Expected: 0x{checksum_calculated:02X}, Got: 0x{checksum:02X}")
            return None
    else:
        print("No response or incomplete data.")
        return None

def write_register(ser, addr, value):
    # Prepare the frame to write the value
    data_l = value & 0xFF
    data_m = (value >> 8) & 0xFF
    data_h = (value >> 16) & 0xFF
    checksum = (((0xA8 + addr + data_l + data_m + data_h) & 0xFF) ^ 0xFF)  # Correct checksum
    frame = bytes([0xA8, addr, data_l, data_m, data_h, checksum])

    print(f"Sent write frame: {[hex(b) for b in frame]}")
    ser.reset_input_buffer()
    ser.write(frame)
    
    # Wait a bit for the write operation to complete
    time.sleep(0.1)  # Adjust if needed based on actual timing
    
# ----------- MAIN -------------
COM_PORT = 'COM3'  # Change this to your correct COM port

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
        
        # Read register 0x1A before writing 0x55 (ensure unlock is done)
        print("\nReading value from register 0x1A before write:")
        read_register(ser, 0x1A)

        # Write 0x55 to register 0x1A to unlock user operation register area
        print("\nWriting 0x55 to register 0x1A (USR_WRPROT) to unlock user register area...")
        write_register(ser, 0x1A, 0x55)

        # Read register 0x18 before writing new value
        print("\nReading value from register 0x18 before write:")
        read_register(ser, 0x18)

        # Write new value (0x000100) to register 0x18
        print("\nWriting 0x000100 to register 0x18...")
        write_register(ser, 0x18, 0x000100)

        # Read register 0x18 after writing new value
        print("\nReading value from register 0x18 after write:")
        read_register(ser, 0x18)

        ser.close()
    else:
        print(f"Failed to open {COM_PORT}")

except serial.SerialException as e:
    print(f"Serial error: {e}")
