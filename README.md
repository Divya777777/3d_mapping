# 3D Mapping Package (ROS 2 Humble)

![](https://github.com/Divya777777/3d_mapping/blob/main/gifs/3d_mapping.gif)

This package contains the simulation environment, configurations, and scripts required to perform 3D SLAM using RTAB-Map and Gazebo Harmonic on a differential drive robot equipped with a RealSense D435 RGB-D camera and LiDAR.

## Overview
The primary goal of this package is to seamlessly bridge the gap between **Gazebo Harmonic** (`gz-transport13`) and **ROS 2 Humble**, specifically addressing real-time message synchronization challenges between high-frequency odometry and high-bandwidth camera streams.

It launches a simulated warehouse environment along with a differential drive robot (`navigation_demo` chassis + RealSense D435), establishes the necessary message bridges, and runs the RTAB-Map SLAM node to generate dense 3D dense point clouds and 2D occupancy grids.

## Key Features & Custom Bridges
- **`gz_camera_bridge.py`**: A custom, highly optimized Python bridge for transporting `gz.msgs.Image` and `gz.msgs.CameraInfo` to ROS 2 in real-time. It uses a decoupled 10 Hz timer-based consumer queue to avoid massive processing delays caused by Python's blocking Protobuf parser, keeping visual frames perfectly synced with simulation odometry (< 0.5s delay).
- **`gz_bridge_relay.py`**: A native transport relay handling `cmd_vel` routing to Gazebo, and bridging the simulation clock (`/clock`), odometry (`/odom`), and extracting `odom -> base_footprint` TF transforms directly from the Gazebo `/odom` ground-truth topic.
- **Odometry Drift Prevention**: Employs physical constraints in the differential drive plugin (`max_linear_acceleration`, `max_angular_velocity`) to prevent infinite torque simulation errors and stop wheels from slipping, guaranteeing accurate odometry mapping.

## Dependencies
- ROS 2 Humble
- Gazebo Harmonic (`gz-sim8`)
- `ros-humble-ros-gz` (Gazebo-ROS bridge utilities)
- `ros-humble-rtabmap-ros`
- `ros-humble-robot-state-publisher`
- `ros-humble-xacro`
- `navigation_demo` (Custom upstream package containing the base `diff_bot.xacro` URDF)
- Python 3 `protobuf`

## Folder Structure

```txt
3d_mapping/
│
├── 3d_mapping/                  # (Python Module)
│   ├── __init__.py
│   ├── gz_bridge_relay.py       
│   └── gz_camera_bridge.py      
│
├── config/                      # Configuration Files
│   └── mapping.rviz             
│
├── launch/                      # Launch Files
│   ├── mapping.launch.py        
│   └── spawn_robot.launch.py    
│
├── urdf/                        # Robot Description
│   ├── mapping_bot.xacro        
│   └── realsense_d435.xacro     
│
├── worlds/                      # Gazebo Worlds
│   └── warehouse.sdf            
│
├── CMakeLists.txt               
├── package.xml                  
└── setup.py                     s
```

## Running the Simulation

**1. Build the Workspace:**
```bash
colcon build --packages-select 3d_mapping
source install/setup.bash
```

**2. Launch the Simulation (Terminal 1):**
This launches Gazebo Harmonic, converts the URDF, spawns the robot, and brings up the custom `gz_camera_bridge` and `gz_bridge_relay` nodes.
```bash
ros2 launch 3d_mapping spawn_robot.launch.py
```

**3. Launch the Mapping Stack (Terminal 2):**
Wait until the Gazebo environment is fully loaded and simulating, then start the RTAB-Map SLAM node and RViz visualization.
```bash
ros2 launch 3d_mapping mapping.launch.py
```

## Common Topics
- `/camera/color/image_raw` - RGB Camera feed (ROS 2)
- `/camera/depth/image_raw` - Depth Camera feed (ROS 2)
- `/odom` - Simulated Odometry (ROS 2)
- `/cmd_vel` - Robot Velocity Command (ROS 2)
- `/rtabmap/cloud_map` - RTAB-Map 3D Dense Map (ROS 2)
- `/map` - 2D Occupancy Grid Projection (ROS 2)
