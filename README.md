
# export_pcd_from_ros2_bag

This repository provides a Python script to extract and export 3D point cloud data from a ROS 2 bag file. The point cloud is typically fused from a ZED stereo camera by enabling the spatial mapping feature and saved in `.pcd` format for visualization in tools such as [CloudCompare](https://www.cloudcompare.org/).

## Features

- Extracts fused point cloud from a ROS 2 topic.
- Converts and exports point cloud data to `.pcd` format.
- Supports visualization in external 3D tools like CloudCompare.

## Prerequisites

Make sure you have the following installed:

- ROS 2 (tested with Humble/Foxy)
- Python 3.8+
- `rosbag2_py`
- `sensor_msgs`



## How to Run

Make sure your ROS 2 environment is sourced and dependencies are installed.

Run the script with:

```bash
python3 export_pointcloud_to_pcd.py zed_slam_bag_20250728_203622
