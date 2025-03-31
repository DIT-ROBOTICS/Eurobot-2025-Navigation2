# Eurobot-2025-Navigation2

## Overview
Eurobot-2025-Navigation2 provides essential navigation capabilities for the DIT Robotics Eurobot 2025 robot. It includes core navigation functions, docking process integration, and various optimizations to enhance autonomous movement and docking performance.

## Features

- **Basic Navigation Functions** – Enables smooth and efficient robot movement using ROS 2 Navigation Stack (Nav2).
- **Docking Process Integration** – Seamlessly integrates the robot's docking and movement workflows.
- **Optimized Navigation Parameters** – Fine-tuned parameters for improved localization, path planning, and obstacle avoidance.
- **Navigation Termination** – Implements the `stopRobot` feature to lock the navigation system when needed.

## Installation

1. Clone this repository:
   ```sh
   git clone https://github.com/DIT-ROBOTICS/Eurobot-2025-Navigation2.git
   ```
2. Navigate to the workspace:
   ```sh
   cd Eurobot-2025-Navigation2
   ```
3. Install dependencies:
   ```sh
   rosdep install --from-paths src --ignore-src -r -y
   ```
4. Build the workspace:
   ```sh
   colcon build --symlink-install
   ```
5. Source the workspace:
   ```sh
   source install/setup.bash
   ```

## How to Use

1. Start the ROS 2 environment:
   ```sh
   source install/setup.bash
   ```
2. Launch the navigation system:
   ```sh
   ros2 launch eurobot_navigation2 bringup.launch.py
   ```
3. To send a goal, use:
   ```sh
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}"
   ```

## Version Updates

- **Installation Adjustments** – Replaced symbolic links with copied files in the install folder.
- **Parameter Optimization** – Enhanced configurations for improved navigation and docking.
- **Bug Fixes** – Resolved multiple known issues.
- **Navigation Termination** – Added the `stopRobot` feature to lock the navigation system.

## Repository Structure
```
Eurobot-2025-Navigation2/
├── src/                    # Source code for navigation
├── launch/                 # Launch files for starting navigation
├── config/                 # Configuration files for navigation parameters
├── maps/                   # Predefined maps for localization
├── scripts/                # Utility scripts
└── README.md               # Project documentation
```

## Contribution
Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create a feature branch.
3. Commit your changes.
4. Submit a pull request.

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.

## Contact
For any issues or inquiries, please open an issue on GitHub or contact the DIT Robotics team.
