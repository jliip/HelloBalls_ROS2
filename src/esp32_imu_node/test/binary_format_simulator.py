#!/usr/bin/env python3

import serial
import time
import struct
import argparse
import math

def main():
    parser = argparse.ArgumentParser(description='Simulate ESP32 binary data format for testing')
    parser.add_argument('--port', type=str, default='/dev/ttyS0', help='Serial port to use')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate')
    parser.add_argument('--rate', type=float, default=50.0, help='Data rate in Hz')
    
    args = parser.parse_args()
    
    try:
        ser = serial.Serial(port=args.port, baudrate=args.baudrate, timeout=1.0)
        print(f"Connected to {args.port} at {args.baudrate} baud")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return
    
    period = 1.0 / args.rate
    counter = 0
    
    print(f"Sending simulated binary IMU data at {args.rate} Hz. Press Ctrl+C to stop.")
    
    try:
        while True:
            start_time = time.time()
            
            # Simulate wheel encoder values that increase over time
            wheel1 = counter * 10
            wheel2 = counter * 11
            
            # Simulate IMU data with some sine wave movement
            t = counter * period
            
            # Acceleration in x, y directions with small sine wave, z with gravity
            accel_x = int(math.sin(t) * 1000)  # Convert to int16 format
            accel_y = int(math.cos(t) * 1000)
            accel_z = int(16384)  # ~1g in the Z direction
            
            # Angular velocity with small sine wave in all axes
            gyro_x = int(math.sin(t * 2) * 500)
            gyro_y = int(math.cos(t * 2) * 500)
            gyro_z = int(math.sin(t * 3) * 200)
            
            # MCU and host states
            mcu_state = counter % 5  # Example state cycling 0-4
            host_state = 1  # Constant host state
            
            # Pack the binary message
            binary_data = bytearray([
                mcu_state,
                host_state,
                (wheel1 >> 24) & 0xFF,  # wheel1 (4 bytes, big endian)
                (wheel1 >> 16) & 0xFF,
                (wheel1 >> 8) & 0xFF,
                wheel1 & 0xFF,
                (wheel2 >> 24) & 0xFF,  # wheel2 (4 bytes, big endian)
                (wheel2 >> 16) & 0xFF,
                (wheel2 >> 8) & 0xFF,
                wheel2 & 0xFF,
            ])
            
            # Append IMU data (16-bit values, big endian)
            binary_data.extend(accel_x.to_bytes(2, byteorder='big', signed=True))
            binary_data.extend(accel_y.to_bytes(2, byteorder='big', signed=True))
            binary_data.extend(accel_z.to_bytes(2, byteorder='big', signed=True))
            binary_data.extend(gyro_x.to_bytes(2, byteorder='big', signed=True))
            binary_data.extend(gyro_y.to_bytes(2, byteorder='big', signed=True))
            binary_data.extend(gyro_z.to_bytes(2, byteorder='big', signed=True))
            
            # Send the binary data
            ser.write(binary_data)
            
            counter += 1
            
            # Calculate sleep time to maintain the desired rate
            elapsed = time.time() - start_time
            sleep_time = max(0, period - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()
