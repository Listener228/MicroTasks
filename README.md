# PROJECT OVERVIEW
## MicroTasks

The project consists of two independent robotics programs: an ESP32-based autonomous cleaning robot and a ROS 2 CatchBot simulation.

### 1. ESP32 Autonomous Robot (C++)

The following tasks were implemented:

1. **ESP32 Configuration:** Configured GPIO pins, motor drivers, servo motor, ultrasonic sensor, and control button using PlatformIO.
2. **Motor Control:** Implemented forward, backward, left, right, and stop functions for the four-wheel robot.
3. **PWM Control:** Configured motor speed control with 1000 Hz frequency and 8-bit resolution.
4. **Cleaning System:** Programmed two rotating brushes to operate during forward movement and stop during other movements.
5. **Distance Measurement:** Integrated the HC-SR04 sensor and implemented distance calculation using ultrasonic signals.
6. **Sensor Filtering:** Added averaging of three measurements to improve distance reading stability.
7. **Servo Scanning:** Configured sensor rotation to 30°, 90°, and 150° for obstacle detection.
8. **Obstacle Avoidance:** Implemented automatic stopping at 40 cm, reversing for 400 ms, comparing left and right distances, and turning toward the clearer direction.
9. **Start/Stop Control:** Implemented push-button operation and motor driver enable/disable logic.

### 2. ROS 2 CatchBot Simulation (Python)

The following tasks were implemented:

1. **ROS 2 Node:** Created the `catch_two_bot` node using Python and `rclpy`.
2. **Robot Communication:** Configured publishers and subscribers for velocity commands and position feedback of three turtles.
3. **Target Movement:** Implemented random movement for two targets with linear speed of 1.0 and variable angular velocity.
4. **Distance Calculation:** Used Euclidean distance to calculate the distance between the main robot and each target.
5. **Target Selection:** Implemented automatic selection of the nearest active target.
6. **Navigation Algorithm:** Used `atan2()` to calculate the steering direction and adjust angular velocity.
7. **Target Tracking:** Configured the main robot to follow targets with a linear speed of 2.0.
8. **Target Catching:** Implemented automatic target removal when the distance becomes less than 0.5 simulation units.
9. **Automatic Respawning:** Used ROS 2 Spawn and Kill services to recreate targets at random coordinates.
10. **Simulation Control:** Implemented a 0.1-second timer for continuous movement, target tracking, and state updates.
