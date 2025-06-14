# RobotMecanum

## `motorController` directory

This directory contains the logic for controlling the robot's motors, including:

| Class Name             | Description                                                                                                   | Uses                        |
|------------------------|---------------------------------------------------------------------------------------------------------------|-----------------------------|
| **motorGearBox**       | Encapsulates the management and control of a single geared motor, including speed and direction control.       | -                           |
| **movementController** | Provides high-level movement commands for the robot (directional movement, in-place rotation, stop).           | **motorGearBox**            |
| **MotionManager**      | Handles incoming movement commands (e.g., from serial input), interprets them, and triggers robot actions.     | **movementController**      |

- **motorGearBox**: This class is responsible for low-level control of a single motor, such as setting speed and direction. It abstracts the hardware details of the motor gearbox.
- **movementController**: This class manages the coordinated movement of all robot motors. It uses multiple `motorGearBox` instances to perform complex maneuvers like moving forward, backward, sideways, diagonally, and rotating in place. It provides a simple interface for high-level movement commands.
- **MotionManager**: This class acts as the main interface for receiving and processing movement commands (for example, from a serial interface). It interprets the commands and calls the appropriate methods of `movementController` to execute the desired movement.

This module abstracts the robot's movement and motor control logic.

## `speedMesurement` directory

This directory contains the logic for measuring and managing the speed of the robot's motors, including:

| Class/Module Name      | Description                                                                                                   | Uses                        |
|-----------------------|---------------------------------------------------------------------------------------------------------------|-----------------------------|
| **rotaryEncoder**     | Handles the reading and processing of rotary encoder signals to determine wheel rotation and speed.            | -                           |
| **hallSensor**        | Manages the acquisition of speed data using Hall effect sensors.                                               | -                           |
| **globalSpeed**       | Aggregates speed data from all sensors and provides global speed information for the robot.                    | **rotaryEncoder**, **hallSensor** |
| **encoderParameter**  | Defines parameters and configuration for encoders (constants, calibration, etc.).                             | -                           |

- **rotaryEncoder**: This module/class is responsible for interfacing with rotary encoders attached to the robot's wheels. It processes encoder pulses to calculate rotation and speed.
- **hallSensor**: This module/class manages Hall effect sensors, which are used as an alternative method for measuring wheel speed.
- **globalSpeed**: This class aggregates speed measurements from all available sensors (rotary encoders and Hall sensors) to provide a unified speed value for the robot. It may also handle filtering and averaging.
- **encoderParameter**: This header defines configuration parameters, constants, and calibration values for the encoders, ensuring accurate speed measurement.

This module abstracts the speed measurement and sensor fusion logic for the robot's movement system.