# G1 Description

Optimized Unitree G1 description with joint state publisher to show in rviz

## Notice

This package changes the base link of Unitree G1 to `torso_link`. So that the external sensors mounted on torso can be visualized directly in RViz without any transformation.

## Usage

### Installation
1. Clone the repository in your ROS workspace:
    ```bash
    git clone https://github.com/ziwenzhuang/g1_description.git
    ```
2. Make sure to install Unitree Dependencies
    - `unitree_go` message definitions
    - `unitree_hg` message definitions
3. Build the package:
    ```bash
    colcon build
    ```
4. Source your workspace:
    ```bash
    source install/setup.bash
    ```

### Running

#### Joint State Publisher

    ```bash
    ros2 run g1_description joint_state_publisher
    ```

    - Add `publish_fake_states:=true` to publish fake states for visualization in RViz without a real robot.

#### RViz with G1 Model
    ```bash
    ros2 launch g1_description display.py
    ```

    - Also, adding `use_fake_states:=true` will use the fake states published by the joint state publisher without a real robot.
