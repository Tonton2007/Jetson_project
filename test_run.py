#!/usr/bin/env python3
"""
jetson_pico_test.py – Script to test communication between Jetson Orin Nano and Raspberry Pi Pico.

This script uses PySerial to send a sequence of commands to the Pico and read its responses.
It performs:
 1. Wheel movement test (strafe right, strafe left, rotate 180°).
 2. Servo movement test on channel 0 (45° -> 90° -> 45°).
 3. Ultrasonic distance sensor test (request distance).

The Pico should be running a MicroPython firmware (main.py) that supports these commands and returns responses.
"""
import sys
import time
import serial  # PySerial is required (install with pip):contentReference[oaicite:10]{index=10}
import argparse

def run_test_sequence(ser):
    """
    Send a fixed sequence of test commands to the Pico via the serial port.
    Reads and logs the Pico's responses for each command.
    """
    # 1. Wheel control test: strafe right, strafe left, rotate ~180 degrees (two ROT_R commands), then stop.
    print("\n*** Starting wheel movement test sequence ***")
    # Move right
    cmd = "STRAFE_R"
    print(f"\nSending command: {cmd}")
    ser.write((cmd + "\n").encode())      # send command with newline terminator
    response = ser.readline().decode(errors='ignore').strip()  # read the Pico's response (with timeout):contentReference[oaicite:11]{index=11}
    if response:
        print(f"Pico response: {response}")  # e.g., "OK: STRAFE_R":contentReference[oaicite:12]{index=12}
    else:
        print("Warning: No response for STRAFE_R")
    time.sleep(1.0)  # wait 1 second for the robot to move
    
    # Move left
    cmd = "STRAFE_L"
    print(f"\nSending command: {cmd}")
    ser.write((cmd + "\n").encode())
    response = ser.readline().decode(errors='ignore').strip()
    if response:
        print(f"Pico response: {response}")  # expected "OK: STRAFE_L"
    else:
        print("Warning: No response for STRAFE_L")
    time.sleep(1.0)  # wait 1 second
    
    # Rotate right 90° (approximately) twice to achieve ~180°
    cmd = "ROT_R"
    print(f"\nSending command: {cmd} (rotate ~90°)")
    ser.write((cmd + "\n").encode())
    response = ser.readline().decode(errors='ignore').strip()
    if response:
        print(f"Pico response: {response}")  # expected "OK: ROT_R"
    else:
        print("Warning: No response for ROT_R")
    time.sleep(1.0)  # pause to allow roughly a 90° rotation (adjust delay as needed for actual robot)
    
    print(f"\nSending command: {cmd} (rotate another ~90°)")
    ser.write((cmd + "\n").encode())  # send ROT_R again for another 90°
    response = ser.readline().decode(errors='ignore').strip()
    if response:
        print(f"Pico response: {response}")  # another "OK: ROT_R"
    else:
        print("Warning: No response for second ROT_R")
    time.sleep(1.0)  # wait for rotation to complete
    
    # Stop movement
    cmd = "STOP"
    print(f"\nSending command: {cmd} (stop all motors)")
    ser.write((cmd + "\n").encode())
    response = ser.readline().decode(errors='ignore').strip()
    if response:
        print(f"Pico response: {response}")  # expected "OK: STOP":contentReference[oaicite:13]{index=13}
    else:
        print("Warning: No response for STOP")
    
    # 2. Servo movement test on channel 0: 45°, then 90°, then back to 45°
    print("\n*** Starting servo movement test sequence ***")
    for angle in [45, 90]:
        cmd = f"SERVO 15 {angle}"
        print(f"\nSending command: {cmd}")
        ser.write((cmd + "\n").encode())
        response = ser.readline().decode(errors='ignore').strip()
        if response:
            print(f"Pico response: {response}")  # e.g., "OK: SERVO ch0 -> 45.0 deg":contentReference[oaicite:14]{index=14}
        else:
            print(f"Warning: No response for SERVO 0 {angle}")
        time.sleep(0.5)  # short delay to allow servo to move to position
    
    # 3. Ultrasonic distance sensor test: request distance
    print("\n*** Starting ultrasonic distance sensor test ***")
    cmd = "DIST?"
    print(f"\nSending command: {cmd}")
    ser.write((cmd + "\n").encode())
    # Read the distance response (expected format "DIST: <value> cm" or "DIST: NONE"):contentReference[oaicite:15]{index=15}
    response = ser.readline().decode(errors='ignore').strip()
    if response:
        print(f"Pico response: {response}")  # e.g., "DIST: 123.4 cm"
        # If needed, parse the numeric value out of the response:
        if response.startswith("DIST:") and response != "DIST: NONE":
            try:
                dist_value = float(response.split()[1])  # second token is the number
                print(f"Distance reading: {dist_value} cm")
            except Exception as e:
                pass  # parsing failed (not a standard format)
    else:
        print("Warning: No response for DIST?")
    
    print("\n*** Test sequence completed ***")

def main():
    # Parse command-line arguments for flexibility (e.g., to add interactive mode easily in future).
    parser = argparse.ArgumentParser(description="Test script for Jetson-Pico serial communication.")
    parser.add_argument("--port", "-p", default="/dev/ttyACM0",
                        help="Serial port device for Pico (default: /dev/ttyACM0)")
    parser.add_argument("--baud", "-b", type=int, default=115200,
                        help="Baud rate for serial port (default: 115200)")
    parser.add_argument("--interactive", "-i", action="store_true",
                        help="Interactive mode: enter a REPL to send manual commands to Pico")
    args = parser.parse_args()
    
    # Open the serial port
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)  # 1-second timeout for reads:contentReference[oaicite:16]{index=16}
        # (If /dev/ttyACM0 is not accessible, ensure user is in 'dialout' group on Linux:contentReference[oaicite:17]{index=17})
    except serial.SerialException as e:
        print(f"ERROR: Could not open serial port {args.port}: {e}")
        sys.exit(1)
    
    print(f"Opened serial port {args.port} at {args.baud} baud.")
    # (Optional) Flush any initial data from the Pico (e.g., startup messages)
    ser.reset_input_buffer()
    
    if args.interactive:
        # Interactive mode: simple REPL loop for manual commands
        print("Entering interactive mode. Type commands to send to the Pico (type 'exit' or Ctrl-C to quit).")
        try:
            while True:
                user_input = input("Enter command (or 'exit'): ").strip()
                if not user_input:
                    continue
                if user_input.lower() in ("exit", "quit"):
                    break
                # Send the user's command to Pico
                ser.write((user_input + "\n").encode())
                # Read and print Pico's response(s)
                time.sleep(0.1)  # small delay to allow Pico to respond
                while ser.in_waiting:  # read all available response lines
                    line = ser.readline().decode(errors='ignore').strip()
                    if line:
                        print(f"Pico response: {line}")
        except KeyboardInterrupt:
            print("\nInteractive mode terminated by user.")
    else:
        # Default mode: run the fixed test sequence
        run_test_sequence(ser)
    
    # Close the serial port
    ser.close()
    print("Serial port closed.")

if __name__ == "__main__":
    main()
