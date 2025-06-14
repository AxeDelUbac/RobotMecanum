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