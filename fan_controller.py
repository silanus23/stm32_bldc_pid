#!/usr/bin/env python3
#
# Fan Controller Interaction Script
#
# Uses two threads to handle simultaneous reading and writing, solving
# the problem of telemetry data interfering with typed commands.

import serial
import threading
import sys

SERIAL_PORT = '/dev/ttyACM1'
BAUD_RATE = 115200
def read_and_print_serial(ser):
    print("--- Telemetry Receiver Started ---")
    try:
        while True:
            line = ser.readline()
            if line:
                print(line.decode('utf-8').strip())
    except serial.SerialException as e:
        print(f"\nError reading from serial port: {e}")
        print("--- Telemetry Receiver Stopped ---")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")

def main():
    print(f"Attempting to connect to {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Successfully connected to {SERIAL_PORT}.")
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}.")
        print(f"Details: {e}")
        print("Please check that the device is connected and you have the correct permissions.")
        sys.exit(1)

    read_thread = threading.Thread(target=read_and_print_serial, args=(ser,), daemon=True)
    read_thread.start()

    print("\n--- Command Sender Ready ---")
    print("Type your command (e.g., 's1600', 'p0.1', 'save') and press Enter.")
    print("Type 'exit' or 'quit' to close.")

    try:
        while True:
            command = input()
            if command.lower() in ['exit', 'quit']:
                print("Exiting...")
                break
            ser.write((command + '\n').encode('utf-8'))
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Closing port.")
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
