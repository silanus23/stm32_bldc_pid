#!/usr/bin/env python3
#
# Fan Controller Interaction Script
#
# This script connects to the STM32 fan controller over a serial port.
# It uses two threads to handle simultaneous reading and writing, which solves
# the problem of telemetry data ("the rain") interfering with typed commands.

import serial
import threading
import sys

# Configuration
SERIAL_PORT = '/dev/ttyACM1'
BAUD_RATE = 115200 # The baud rate doesn't matter for VCP, but it's good practice to set it.

def read_and_print_serial(ser):
    """
    This function runs in a background thread.
    Its only job is to continuously read lines from the serial port
    and print them to the console.
    """
    print("--- Telemetry Receiver Started ---")
    try:
        while True:
            # Read a line from the serial port, which blocks until a newline is received
            line = ser.readline()
            if line:
                # Decode the bytes into a string and strip any whitespace
                print(line.decode('utf-8').strip())
    except serial.SerialException as e:
        print(f"\nError reading from serial port: {e}")
        print("--- Telemetry Receiver Stopped ---")
    except Exception as e:
        # This will catch other errors, like if the script is force-closed
        print(f"\nAn unexpected error occurred: {e}")


def main():
    """
    Main function to handle the connection and user input.
    """
    print(f"Attempting to connect to {SERIAL_PORT}...")

    try:
        # Establish the serial connection
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Successfully connected to {SERIAL_PORT}.")
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}.")
        print(f"Details: {e}")
        print("Please check that the device is connected and you have the correct permissions.")
        sys.exit(1)

    # Create and start the background thread for reading telemetry
    # 'daemon=True' means the thread will automatically exit when the main program does
    read_thread = threading.Thread(target=read_and_print_serial, args=(ser,), daemon=True)
    read_thread.start()

    print("\n--- Command Sender Ready ---")
    print("Type your command (e.g., 's1600', 'p0.1', 'save') and press Enter.")
    print("Type 'exit' or 'quit' to close.")

    try:
        while True:
            # The main thread waits here for the user to type a command
            command = input()

            if command.lower() in ['exit', 'quit']:
                print("Exiting...")
                break

            # Send the command to the STM32
            # We encode the string into bytes and add a newline character
            ser.write((command + '\n').encode('utf-8'))

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Closing port.")
    finally:
        # Ensure the serial port is closed gracefully
        if ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
