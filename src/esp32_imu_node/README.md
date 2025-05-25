# ESP32 IMU Node

This ROS2 node interfaces with an ESP32 microcontroller that provides IMU data through a serial connection.
The node supports both JSON and binary data formats.

## Features

- Reads IMU data from an ESP32 via a serial port
- Publishes standard sensor_msgs/Imu messages
- Supports both JSON and binary data formats
- Configurable parameters via ROS parameters
- Auto-reconnect capability if the serial connection is lost

## Installation

```bash
cd ~/ros2_ws
colcon build --packages-select esp32_imu_node
source install/setup.bash
```

## Usage

Launch the node using the provided launch file:

```bash
ros2 launch esp32_imu_node esp32_imu.launch.py
```

You can override the default parameters:

```bash
ros2 launch esp32_imu_node esp32_imu.launch.py serial_port:=/dev/ttyUSB0 use_binary_format:=true
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| serial_port | string | /dev/ttyS1 | Serial port connected to the ESP32 |
| baud_rate | int | 115200 | Serial communication baud rate |
| frame_id | string | imu_link | Frame ID for IMU messages |
| publish_rate | float | 100.0 | Target publish rate in Hz |
| connect_retry_period | float | 5.0 | Seconds between connection attempts |
| use_binary_format | bool | true | Use binary format (true) or JSON format (false) |

## Data Format

### Binary Format

The binary format expects 22 bytes with the following structure:

| Bytes | Type | Description |
|-------|------|-------------|
| 0 | uint8 | MCU state |
| 1 | uint8 | Host state |
| 2-5 | uint32 (big endian) | Wheel 1 distance |
| 6-9 | uint32 (big endian) | Wheel 2 distance |
| 10-11 | int16 (big endian) | Acceleration X |
| 12-13 | int16 (big endian) | Acceleration Y |
| 14-15 | int16 (big endian) | Acceleration Z |
| 16-17 | int16 (big endian) | Gyroscope X |
| 18-19 | int16 (big endian) | Gyroscope Y |
| 20-21 | int16 (big endian) | Gyroscope Z |

Notes:
- Accelerometer values need to be divided by 16384.0 and multiplied by 9.81 to get m/s²
- Gyroscope values need to be divided by 131.0 and multiplied by π/180 to get rad/s

### JSON Format

The JSON format expects a string with the following structure:

```json
{
  "accel_x": 0.0,
  "accel_y": 0.0,
  "accel_z": 9.81,
  "gyro_x": 0.0,
  "gyro_y": 0.0,
  "gyro_z": 0.0,
  "quat_w": 1.0,
  "quat_x": 0.0,
  "quat_y": 0.0,
  "quat_z": 0.0
}
```

## Testing

The package includes test scripts to simulate both data formats:

### Binary Format Simulator

```bash
cd ~/ros2_ws
./src/esp32_imu_node/test/binary_format_simulator.py --port /dev/ttyS0 --baudrate 115200 --rate 50
```

### JSON Format Simulator

```bash
cd ~/ros2_ws
./src/esp32_imu_node/test/json_format_simulator.py --port /dev/ttyS0 --baudrate 115200 --rate 50
```

## License

Apache License 2.0
