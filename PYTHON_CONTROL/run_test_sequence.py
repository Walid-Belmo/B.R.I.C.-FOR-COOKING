import serial
import serial.tools.list_ports
import time
import sys

def get_serial_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found!")
        return None
    
    # Filter for likely candidates (often CH340 or CP210x on Windows)
    for p in ports:
        if "USB" in p.description or "COM" in p.description:
             # Just return the first one found for simplicity in this script
             return p.device
    return ports[0].device

def send_command(ser, pulses):
    # Format: s1-1500\n
    for i, pulse in enumerate(pulses):
        cmd = f"s{i+1}-{pulse}\n"
        ser.write(cmd.encode())
        time.sleep(0.05) # Small delay to not flood
    print(f"   -> Sent pulses: {pulses}")

def main():
    print("==========================================")
    print("   ROBOT ARM DEBUG SEQUENCE")
    print("==========================================")
    
    port = get_serial_port()
    if not port:
        print("Error: Could not find serial port.")
        return

    print(f"Connecting to {port} at 115200 baud...")
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2) # Wait for ESP32 reset
    except Exception as e:
        print(f"Error connecting: {e}")
        return

    # SCENARIO 1
    print("\n--- TEST 1: NEUTRAL POSITION ---")
    print("Moving robot to [1500, 1500, 1500, 1500]...")
    send_command(ser, [1500, 1500, 1500, 1500])
    print("\nACTION REQUIRED:")
    print("1. Measure Height (Z) from table to tip.")
    print("   Expected: ~35.1 cm")
    print("2. Measure Extension (Y) from center to tip.")
    print("   Expected: ~4.0 cm")
    
    input("\nPress ENTER to continue to Test 2...")

    # SCENARIO 2
    print("\n--- TEST 2: SHOULDER HORIZONTAL ---")
    print("Moving robot to [1500, 2076, 1500, 1500]...")
    send_command(ser, [1500, 2076, 1500, 1500])
    print("\nACTION REQUIRED:")
    print("1. Check if the upper arm (J2-J3) is visually HORIZONTAL.")
    print("2. Measure Height (Z) from table to tip.")
    print("   Expected: ~9.8 cm")
    
    input("\nPress ENTER to finish and detach...")
    ser.close()
    print("Done.")

if __name__ == "__main__":
    main()
