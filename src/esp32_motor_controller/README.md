# esp32_motor_controller

## Overview
The `esp32_motor_controller` package is designed for controlling motors via an ESP32 microcontroller connected to a RDK X5 mini PC. It communicates over UART serial at `/dev/ttyS1` with a baud rate of 115200 and ODD parity check. The package sends motor control messages in the format "status,motor1_speed,motor2_speed" (e.g., "1,1000,1000").

## Installation
To install the package, follow these steps:

1. Clone the repository:
   ```
   git clone <repository_url>
   cd esp32_motor_controller
   ```

2. Build the package:
   ```
   colcon build
   ```

3. Source the setup file:
   ```
   source install/setup.bash
   ```

## Usage
To run the motor controller node, use the following command:

```
ros2 launch esp32_motor_controller motor_controller.launch.py
```

This will start the motor controller node with the parameters defined in the `motor_controller.yaml` configuration file.

## Configuration
The configuration file `motor_controller.yaml` contains parameters for the motor controller node, including:

- `serial_port`: The serial port to which the ESP32 is connected (default: `/dev/ttyS1`).
- `baud_rate`: The baud rate for the serial connection (default: `115200`).
- `motor1_speed`: The speed setting for motor 1.
- `motor2_speed`: The speed setting for motor 2.

## Testing
The package includes tests for code style and compliance with PEP 257. To run the tests, use:

```
pytest
```

## License
This package is licensed under the MIT License. See the LICENSE file for more details.

## Maintainers
- Your Name <your.email@example.com>