#!/usr/bin/env python3

import serial
import time
import json
import argparse
import math

def main():
    parser = argparse.ArgumentParser(description='Simulate ESP32 JSON data format for testing')
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
    
    print(f"Sending simulated JSON IMU data at {args.rate} Hz. Press Ctrl+C to stop.")
    
    try:
        while True:
            start_time = time.time()
            
            # Simulate IMU data with some sine wave movement
            t = counter * period
            
            # Acceleration in x, y directions with small sine wave, z with gravity
            accel_x = math.sin(t) * 0.5  # m/s²
            accel_y = math.cos(t) * 0.5  # m/s²
            accel_z = 9.81  # gravity in the Z direction
            
            # Angular velocity with small sine wave in all axes
            gyro_x = math.sin(t * 2) * 0.2  # rad/s
            gyro_y = math.cos(t * 2) * 0.2  # rad/s
            gyro_z = math.sin(t * 3) * 0.1  # rad/s
            
            # Create JSON data
            data = {
                "accel_x": accel_x,
                "accel_y": accel_y,
                "accel_z": accel_z,
                "gyro_x": gyro_x,
                "gyro_y": gyro_y,
                "gyro_z": gyro_z,
                "quat_w": 1.0,
                "quat_x": 0.0,
                "quat_y": 0.0,
                "quat_z": 0.0
            }
            
            # Convert to JSON string and add newline
            json_str = json.dumps(data) + "\n"
            
            # Send the JSON string
            ser.write(json_str.encode('utf-8'))
            
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
