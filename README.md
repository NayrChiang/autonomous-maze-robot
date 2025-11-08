# Autonomous Maze Navigation Robot

A comprehensive autonomous robot system for maze navigation, localization, and block manipulation using Arduino, MATLAB, and probability-based algorithms. This project demonstrates integration of multiple sensors, real-time control, and autonomous decision-making in a complex environment.

## Project Overview

**Course**: MIE444 - Mechatronics  
**Institution**: University of Toronto  
**Term**: Fall 2022

### Objective

Design and build an autonomous robot capable of navigating a maze, localizing its position using probability-based algorithms, avoiding obstacles, detecting blocks, and manipulating objects to complete assigned tasks.

## Key Features

### Multi-Sensor Data Fusion
- **7 Ultrasonic Sensors (HC-SR04)**: Front, left, right, back, and 45° angle sensors for comprehensive obstacle detection
- **2 IR Sensors**: Block detection and floor pattern recognition for localization
- Multi-sample averaging (5 samples per reading) for noise reduction
- Real-time sensor data fusion for robust environment sensing

### Probability-Based Localization
- Monte Carlo localization algorithm in 32×16 grid maze (512 possible locations)
- IR sensor for color tile recognition
- Real-time position tracking with probability distribution updates
- 10% probability threshold for successful localization confirmation
- Heading initialization and correction at block 14 using unique distance patterns

### Autonomous Navigation
- Perfect obstacle avoidance with collision-free navigation
- Four heading directions (W, A, S, D) for 2D IR localization
- Angle correction logic comparing current and previous step sensor readings
- Wall distance thresholds: front (8cm), sides (4cm), back (5cm)
- Fixed path planning for 32 block locations with configurable loading/unloading sequences

### Block Detection and Manipulation
- Ultrasonic sensor scanning sequence (90-100° rotation in 5° increments)
- Detection criteria: Bottom sensor < 50 cm AND absolute difference between front and bottom sensor > 10 cm
- 3D-printed mechanical gripper controlled by two SG90 servo motors
- Joint servo for elevation control, jaw servo for opening/closing
- Pickup and delivery sequences with precise positioning

### MATLAB-Arduino Integration
- Real-time control via Bluetooth (HC-05, 9600 baud)
- Serial command protocol for motor and servo control
- Real-time probability distribution visualization
- Simplified testing and troubleshooting workflow

## Hardware Components

### Microcontrollers
- **Arduino Uno** (2 boards): Separate boards for sensors and motor/servo control

### Sensors
- **7× Ultrasonic Sensors (HC-SR04)**: Obstacle detection at multiple angles
- **2× IR Sensors**: Block detection and floor pattern recognition

### Actuators
- **DC Motors**: With H-bridge (L298N) for omnidirectional movement
- **2× Servo Motors (SG90)**: 
  - Gripper servo: Controls opening/closing (45-90°)
  - Lift servo: Controls elevation (20-140°)
- **Omni Wheels**: For omnidirectional movement

### Communication
- **HC-05 Bluetooth Module**: 9600 baud serial communication

### Chassis
- Custom 3D-printed design with integrated sensor mounts
- Redesigned from 9-inch to 6.5-inch diameter (top layer) for improved sensor accuracy
- Multi-layer assembly for component organization

## Software Architecture

### Arduino Firmware
- **Arduino_for_Sensors.ino**: Sensor data acquisition and processing
- **Arduino_for_Motor_&_Servo.ino**: Motor and servo control
- Serial command protocol for MATLAB communication

### MATLAB Control Algorithms
- **Matlab_Real_ObsAvoid_HeadingInit_LoadDropOffZone.m**: Main control algorithm
- **Matlab_Real_Block_Detection_PickUp.m**: Block detection and manipulation
- **Matlab_Simulator_ObsAvoid_HeadingInit_LoadDropOffZone.m**: Simulation for testing

### Key Algorithms
- Probability-based localization (Monte Carlo methods)
- Obstacle avoidance with angle correction
- Block detection using ultrasonic scanning
- Path planning and navigation

## Technical Implementation

### Sensor Integration
- Multi-sample averaging (5 samples) for noise reduction
- Distance calculation: ping time to distance (0.017 cm/μs)
- Sensor threshold calibration at various maze locations
- Multi-sensor data fusion for comprehensive environment sensing

### Obstacle Avoidance
- Four heading directions defined for 2D IR localization
- Angle correction logic: Comparison of current step (U3) and previous step (U3') side sensor readings
- If U3 < U3': Rotate counterclockwise to correct angle
- If U3 > U3': Rotate clockwise
- Perfect collision-free navigation achieved

### Localization
- Probability distribution initialization: uniform (1/512 per cell)
- Probability updates: Measurement compared with actual color tile map
- Probability threshold: 10% for confirmation of successful localization
- Threshold-based path switching: switches to fixed path when probability > 10%
- Heading management: Updated by adding/subtracting 90° when robot makes turns

### Block Manipulation
- Initial scan: Shifts left and right to check if block is in straight line of sight
- Detailed scan: 90-100° rotation in 5° increments once inside loading zone
- Pickup sequence: Abort rotation → Drive forward → Open and lower gripper → Close and raise
- Delivery: Navigate to unloading zone → Lower lift → Open gripper → Raise lift → Close gripper

## Project Structure

```
autonomous-maze-robot/
├── arduino-code/
│   ├── Arduino_for_Motor_&_Servo/
│   │   └── Arduino_for_Motor_&_Servo.ino
│   └── Arduino_for_Sensors/
│       └── Arduino_for_Sensors.ino
├── matlab-code/
│   ├── Matlab_Real_Block_Detection_PickUp/
│   │   └── Matlab_Real_Block_Detection_PickUp.m
│   ├── Matlab_Real_ObsAvoid_HeadingInit_LoadDropOffZone/
│   │   └── Matlab_Real_ObsAvoid_HeadingInit_LoadDropOffZone.m
│   └── Matlab_Simulator_ObsAvoid_HeadingInit_LoadDropOffZone/
│       └── Matlab_Simulator_ObsAvoid_HeadingInit_LoadDropOffZone.m
├── cad-models/
│   └── Rover 3D CAD Model/
│       ├── Rover.SLDASM
│       ├── Base - 7 in.SLDPRT
│       ├── Second - 7 in.SLDPRT
│       ├── Third - 7 in.SLDPRT
│       ├── Top Layer - 6 in.SLDPRT
│       └── [other component files]
├── MIE444 Project Final Report.pdf
└── README.md
```

## Installation & Setup

### Prerequisites
- Arduino IDE
- MATLAB (with Arduino Support Package)
- HC-05 Bluetooth module
- Required hardware components (see Hardware Components section)

### Hardware Setup
1. Assemble robot chassis according to CAD models
2. Install sensors and actuators
3. Connect Arduino boards and Bluetooth module
4. Power system setup

### Software Setup
1. Install Arduino IDE and required libraries
2. Upload sensor code to Arduino_for_Sensors board
3. Upload motor/servo code to Arduino_for_Motor_&_Servo board
4. Install MATLAB Arduino Support Package
5. Configure Bluetooth connection in MATLAB
6. Run MATLAB control scripts

## Usage

### Running the Robot
1. Power on both Arduino boards
2. Establish Bluetooth connection between MATLAB and HC-05 module
3. Run main MATLAB control script: `Matlab_Real_ObsAvoid_HeadingInit_LoadDropOffZone.m`
4. Monitor probability distribution and sensor readings in real-time
5. Robot will autonomously navigate, localize, and complete tasks

### Simulation
- Use `Matlab_Simulator_ObsAvoid_HeadingInit_LoadDropOffZone.m` for testing algorithms before real robot implementation
- Adjust sensor error settings for proof of concept testing

## Final Results

### Obstacle Avoidance
✅ **Met requirement**: Rover did not need human assistance, no wall collisions, made minor heading adjustments  
⚠️ **Trade-off**: Speed was slow due to accuracy trade-off, unable to complete all tasks within time limit

### Localization
✅ **Tested in demonstration**: Rover started at block 30 with heading ~170°, followed path to loading zone, arrived at block 1 (loading zone) within time constraint

### Block Detection
✅ **Functional**: Trial 2 - Rover placed at exterior of loading zone (block 17), successfully detected load at center of block 1

### Block Manipulation
⚠️ **Limited**: Detection logic functional, but pickup/delivery failed due to gripper design limitations:
- Insufficient torque (single 12V battery)
- Weak connection (press-fit enlarged after use)
- Size limitations (2.64 inches jaw distance)
- Motor drift during approach

## Key Achievements

- Successfully integrated 9 sensors (7 ultrasonic + 2 IR) with multi-sample averaging
- Implemented probability-based localization algorithm (Monte Carlo) with 10% threshold
- Developed autonomous navigation system with perfect obstacle avoidance
- Designed angle correction logic for accurate localization
- Achieved reliable block detection using ultrasonic sensor scanning
- Integrated MATLAB-Arduino system via Bluetooth for real-time control
- Implemented fixed path planning system for 32 block locations
- Successfully demonstrated localization and navigation in competition
- Designed and fabricated complete robot chassis using SolidWorks CAD
- Redesigned rover for improved sensor accuracy and performance

## Lessons Learned

- **Time Management**: Better allocation needed for rover design, sensor adjustment, and code integration
- **Gripper Design**: Increase size for tolerance, individual power supply for servos, more reliable connection method
- **Code Organization**: Version control (GitHub) would help with code merging and communication
- **Rover Design**: Initial 9-inch diameter had sensors too close to walls; redesign to 6.5-inch improved performance
- **Bluetooth Implementation**: Significant achievement - simplified testing and enabled simultaneous MATLAB/Arduino testing

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- MIE444 course instructors and TAs
- University of Toronto for providing resources and facilities
- Team members who contributed to the project
