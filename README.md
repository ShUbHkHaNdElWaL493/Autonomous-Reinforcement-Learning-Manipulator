# Autonomous Reinforcement Learning Manipulator

**Autonomous Reinforcement Learning Manipulator** is a project focused on developing an autonomous robotic manipulator using reinforcement learning techniques.

## Features

- Implements reinforcement learning algorithms for robotic manipulation.
- Searches for the maximum rewarding trajectory for final gripper state from initial gripper state
- Modular code structure for easy maintenance and scalability.

## Technologies Used

- **Python**
- **ROS2 Jazzy**
- **Gazebo Sim Harmonic**

## Getting Started

### Prerequisites

- Python 3.x
- ROS2 Jazzy
- Gazebo Sim Harmonic

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/ShUbHkHaNdElWaL493/Autonomous-Reinforcement-Learning-Manipulator.git
   ```

2. Navigate to the project directory:
   ```bash
   cd Autonomous-Reinforcement-Learning-Manipulator
   ```

3. Install requirements:
   ```bash
   pip install -r requirements.txt
   ```

4. Build the project using colcon build:
   ```bash
   colcon build --packages-select manipulator
   source install/setup.bash
   colcon build --packages-select controller
   ```

### Usage

1. Run the 'train' launch file to train the model:
   ```bash
   ros2 launch controller train.launch.py
   ```

2. Build the project again to load the model:
   ```bash
   colcon build --packages-select controller
   ```

3. Run the 'control' launch file to control the manipulator based on a given gripper position:
   ```bash
   ros2 launch controller control.launch.py
   ```

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request for any enhancements or bug fixes.