# A* Algorithm for TurtleBot3 Waffle

## Introduction
This README guides you through running an A* algorithm implementation tailored for the TurtleBot3 Waffle robot. The program navigates the robot from a start node to a goal node, considering non-holonomic constraints and avoiding obstacles.

## Prerequisites
Ensure you have the following installed:
- Python 3.8 or newer
- ROS 2 Galactic
- Ubuntu 20.04
- OpenCV-Python
- Numpy
- rclpy

You can install Python dependencies using pip:
```bash
pip install numpy opencv-python matplotlib
```
# Part -1

## Input Interface
* The program prompts you for the following inputs:

- Clearance: The minimum distance from obstacles, specified in centimeters.
- Start Node: The (X, Y) coordinates and the orientation angle (theta) of the start position in centimeters.
- Goal Node: The (X, Y) coordinates of the goal position in centimeters.
- RPM1 and RPM2: The RPM values for the robot's wheels.

Example values used for the visualization present in the video are mentioned below
```bash
Enter the clearance of the obstacles: 5
Enter the x-coordinate of the start node(Xs) in cm: 50
Enter the y-coordinate of the start node(Ys) in cm: 100
Enter the angle of the start_node: 0
Enter the x-coordinate of the goal node(Xg)in cm: 570
Enter the y-coordinate of the goal node(Yg) in cm: 170
Enter the RPM 1 : 25
Enter the RPM 2 : 30
```

# How It Works
* Visualization and Tree Search
The program utilizes OpenCV to draw the environment using the given map, obstacles, and the search tree as it searches for the optimal path. Each node expansion is visualized, with the tree covering the navigatable space until the goal node is reached within the threshold.

# Final Path
Upon finding the optimal path, the program traces back from the goal node to the start node, highlighting the path on the visualization. This path represents the sequence of movements (linear and angular velocities) the robot needs to follow to reach the goal.

# Output
The program outputs a video file a_star_varun_lakshmanan_sai_jagadeesh_muralikrishnan.mp4 showing the search process and the final path. Additionally, it prints the runtime and confirms whether an optimal path was found at the end.

# Part-2

Terminal Interface instructions
* Creata a ros 2 workspace for the the source file to place inside it 
```bash
mkdir -p <project_workspace>/src
cd <project_workspace>
source /opt/ros/galactic/setup.bash
colcon build 
source install/setup.bash
```
To launch the environment
```bash
ros2 launch turtlebot3_project3 competition_world.launch
```
To launch the ros script for A* Path Planning in scripts file
```bash
ros2 run turtlebot3_project3 ros.py
```

Input Interface
The program prompts for the following inputs:

Clearance: Minimum distance from any obstacle, in centimeters.
Start Node: (X, Y) coordinates and orientation angle (theta) of the start position in centimeters.
Goal Node: (X, Y) coordinates of the goal position in centimeters.
RPM1 and RPM2: Wheel RPM values.

```bash
Enter the clearance of the obstacles: 5
Enter the x-coordinate of the start node(Xs) in cm: 50
Enter the y-coordinate of the start node(Ys) in cm: 100
Enter the angle of the start node: 0
Enter the x-coordinate of the goal node(Xg)in cm: 570
Enter the y-coordinate of the goal node(Yg) in cm: 170
Enter the RPM 1: 25
Enter the RPM 2: 50
```
# How It Works
- The implementation utilizes the A* algorithm to calculate an optimal path considering the robot's motion constraints. 
- It generates a tree of possible moves from the start node, avoiding obstacles until it reaches the goal node. 
- The optimal path is then backtracked from the goal to the start, providing the required velocities at each step.
These velocities has been calculated with the formula
```bash
ul = RPM1*R*(2*np.pi/60)
ur = RPM2*R*(2*np.pi/60)
linear_velocitiy = 0.5*(ul+ur)
angular_vel = (ul-ur)/(L)
```
- The Publisher class publishes the calculated linear.x and angular.z velocities to the robot via ROS 2, enabling real or simulated movement.
- Upon successful execution, the program will:

Prints the runtime and confirmation of path finding in the console.
Generates a video a_star_varun_lakshmanan_sai_jagadeesh_muralikrishnan.mp4 showing the node exploration and final path.
Publishs linear and angular velocities to the TurtleBot3 via ROS 2 and there by observed the Waffle robot followuing the path in the Gazebo.

