#!/usr/bin/env python3
#
# Fan Controller Interaction Script
#
# Uses two threads to handle simultaneous reading and writing, solving
# the problem of telemetry data interfering with typed commands.
#
#   python3 fan_controller.py                 # auto-detect the board
#   python3 fan_controller.py --port /dev/ttyACM1

import argparse
import sys
import threading

import serial
import serial.tools.list_ports

USB_VID = 0x0483
USB_PID = 0x5740    # STM32 USB CDC (the ST-LINK's own port is 0x374B)


def find_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == USB_VID and p.pid == USB_PID:
            return p.device
    return None


def read_and_print_serial(ser, stop):
    print("--- Telemetry Receiver Started ---")
    while not stop.is_set():
        try:
            line = ser.readline()
        except (serial.SerialException, OSError, TypeError) as e:
            if not stop.is_set():
                print(f"\nError reading from serial port: {e}")
                print("--- Telemetry Receiver Stopped ---")
                stop.set()
            return
        if line:
            print(line.decode('utf-8', errors='replace').strip(), flush=True)


def main():
    ap = argparse.ArgumentParser(description="Interactive console for the STM32 PID fan controller")
    ap.add_argument("--port", help="serial port (default: auto-detect the STM32 USB CDC device)")
    args = ap.parse_args()

    port = args.port or find_port()
    if port is None:
        print("Error: STM32 fan controller not found. Is the micro-USB (CN5) cable connected?")
        sys.exit(1)

    print(f"Attempting to connect to {port}...")
    try:
        # USB CDC ignores the baud rate; pyserial just needs a value
        ser = serial.Serial(port, 115200, timeout=0.5)
        print(f"Successfully connected to {port}.")
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {port}.")
        print(f"Details: {e}")
        print("Please check that the device is connected and that your user is in the 'dialout' group.")
        sys.exit(1)

    stop = threading.Event()
    read_thread = threading.Thread(target=read_and_print_serial, args=(ser, stop), daemon=True)
    read_thread.start()

    print("\n--- Command Sender Ready ---")
    print("Type your command (e.g., 's1600', 'p0.1', 'save') and press Enter.")
    print("Type 'exit' or 'quit' to close.")

    try:
        while not stop.is_set():
            command = input()
            if command.lower() in ['exit', 'quit']:
                print("Exiting...")
                break
            try:
                ser.write((command + '\n').encode('utf-8'))
            except (serial.SerialException, OSError) as e:
                print(f"Error writing to serial port: {e}")
                break
    except (KeyboardInterrupt, EOFError):
        print("\nClosing port.")
    finally:
        stop.set()
        read_thread.join(timeout=1.0)
        if ser.is_open:
            ser.close()
            print("Serial port closed.")


if __name__ == "__main__":
    main()
