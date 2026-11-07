#!/usr/bin/env python3
"""
uart_mnist_host.py -- Host PC interface for sending 28x28 MNIST images over UART
                      to the Basys3 FPGA RISC-V CNN SoC and receiving predictions.

Requirements:
  pip install pyserial
"""

import sys
import time
import argparse

try:
    import serial
except ImportError:
    serial = None


def send_image(port, baudrate, image_path):
    if serial is None:
        print("ERROR: pyserial is not installed. Run: pip install pyserial")
        sys.exit(1)

    print(f"Connecting to FPGA on {port} @ {baudrate} baud...")
    ser = serial.Serial(port, baudrate, timeout=3)
    time.sleep(1)  # Wait for connection reset

    # Read image bytes (784 bytes)
    with open(image_path, "rb") as f:
        img_bytes = f.read(784)

    if len(img_bytes) < 784:
        print(f"ERROR: Image file must contain at least 784 bytes (found {len(img_bytes)})")
        sys.exit(1)

    print(f"Streaming 784 pixel bytes to FPGA...")
    ser.write(img_bytes)
    ser.flush()

    print("Waiting for SoC classification response...")
    start_time = time.time()
    while time.time() - start_time < 5.0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(f"  [FPGA] {line}")
            if "Predicted Digit" in line:
                print("\n>>> Recognition Completed! <<<")

    ser.close()


def main():
    parser = argparse.ArgumentParser(description="Host PC UART driver for RISC-V MNIST SoC")
    parser.add_argument("--port", default="/dev/ttyUSB1", help="Serial port (e.g., /dev/ttyUSB1 or COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--image", required=False, help="Binary or hex image file (784 bytes)")
    args = parser.parse_args()

    if args.image:
        send_image(args.port, args.baud, args.image)
    else:
        print("Usage: python3 uart_mnist_host.py --port /dev/ttyUSB1 --image sample_7.bin")

if __name__ == "__main__":
    main()
