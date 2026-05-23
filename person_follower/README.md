# Person Follower Package

## Overview
The `person_follower` package is designed for a robot to autonomously follow a person using ROS (Robot Operating System). It utilizes various sensors and algorithms to detect and track a person, allowing the robot to navigate its environment while maintaining a specified distance from the target.

## Installation
To install the `person_follower` package, follow these steps:

1. Clone the repository:
   ```
   git clone <repository-url>
   cd <repository-directory>
   ```

2. Install the required dependencies:
   ```
   pip install -r requirements.txt
   ```

3. Build the package:
   ```
   cd ~/catkin_ws
   catkin_make
   ```

4. Source the workspace:
   ```
   source devel/setup.bash
   ```

## Usage
To run the person follower node, use the provided launch file:

```
roslaunch person_follower person_follower.launch
```

This will start the necessary nodes and parameters for the person follower functionality.

## Features
- Detects and tracks a person using sensor data.
- Controls the robot's movement to follow the detected person.
- Configurable parameters for following distance and speed.

## Contributing
Contributions are welcome! Please submit a pull request or open an issue for any enhancements or bug fixes.

## License
This project is licensed under the MIT License. See the LICENSE file for details.