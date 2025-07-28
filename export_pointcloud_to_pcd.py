#!/usr/bin/env python3

import os
import sys
import argparse
import numpy as np
from pathlib import Path

# ROS2 imports
import rclpy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import rosbag2_py

def get_rosbag_options(path, serialization_format='cdr'):
    """Get ROS bag options for reading."""
    return rosbag2_py.StorageOptions(
        uri=path,
        storage_id='sqlite3'
    )

def get_converter_options(serialization_format='cdr'):
    """Get converter options for reading."""
    return rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format
    )

def pointcloud2_to_pcd(points, colors=None, filename="pointcloud.pcd"):
    """
    Convert point cloud data to PCD format and save to file.
    
    Args:
        points: numpy array of shape (N, 3) with x, y, z coordinates
        colors: numpy array of shape (N, 3) with RGB values (0-255) or None
        filename: output PCD filename
    """
    num_points = len(points)
    
    # Create PCD header
    pcd_header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z"""
    
    if colors is not None:
        pcd_header += " rgb"
    
    pcd_header += f"""
SIZE 4 4 4"""
    
    if colors is not None:
        pcd_header += " 4"
    
    pcd_header += f"""
TYPE F F F"""
    
    if colors is not None:
        pcd_header += " F"
    
    pcd_header += f"""
COUNT 1 1 1"""
    
    if colors is not None:
        pcd_header += " 1"
    
    pcd_header += f"""
WIDTH {num_points}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {num_points}
DATA ascii
"""
    
    # Write header
    with open(filename, 'w') as f:
        f.write(pcd_header)
        
        # Write point data
        for i in range(num_points):
            line = f"{points[i, 0]:.6f} {points[i, 1]:.6f} {points[i, 2]:.6f}"
            if colors is not None:
                # Convert RGB to PCD format (packed into float)
                r, g, b = colors[i]
                rgb_packed = int(r) << 16 | int(g) << 8 | int(b)
                line += f" {rgb_packed}"
            f.write(line + "\n")
    
    print(f"Saved point cloud with {num_points} points to {filename}")

def extract_pointcloud_from_bag(bag_path, topic_name, output_dir="pcd_exports"):
    """
    Extract point cloud data from ROS2 bag file and save as PCD files.
    
    Args:
        bag_path: Path to the ROS2 bag file
        topic_name: Name of the point cloud topic
        output_dir: Directory to save PCD files
    """
    # Create output directory
    Path(output_dir).mkdir(exist_ok=True)
    
    # Get bag options
    storage_options = get_rosbag_options(bag_path)
    converter_options = get_converter_options()
    
    # Open bag
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Get topic types
    topic_types = reader.get_all_topics_and_types()
    topic_type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
    
    print(f"Available topics:")
    for topic_name_available in topic_type_map:
        print(f"  - {topic_name_available}: {topic_type_map[topic_name_available]}")
    
    if topic_name not in topic_type_map:
        print(f"Error: Topic '{topic_name}' not found in bag file!")
        return
    
    print(f"\nExtracting point clouds from topic: {topic_name}")
    
    # Process messages
    message_count = 0
    while reader.has_next():
        (topic_name_msg, data, t) = reader.read_next()
        
        if topic_name_msg == topic_name:
            # Deserialize the message
            msg = deserialize_message(data, PointCloud2)
            
            # Extract point cloud data
            points, colors = extract_points_from_pointcloud2(msg)
            
            if len(points) > 0:
                # Create filename with timestamp
                timestamp = t  # t is already the timestamp in nanoseconds
                filename = f"{output_dir}/pointcloud_{message_count:04d}_{timestamp}.pcd"
                
                # Save as PCD
                pointcloud2_to_pcd(points, colors, filename)
                message_count += 1
                
                print(f"  Message {message_count}: {len(points)} points")
            else:
                print(f"  Message {message_count}: No valid points found")
    
    print(f"\nExtraction complete! Saved {message_count} PCD files to '{output_dir}' directory.")

def extract_points_from_pointcloud2(msg):
    """
    Extract points and colors from PointCloud2 message.
    
    Args:
        msg: PointCloud2 message
        
    Returns:
        points: numpy array of shape (N, 3) with x, y, z coordinates
        colors: numpy array of shape (N, 3) with RGB values (0-255) or None
    """
    # Get point cloud data
    points = []
    colors = []
    
    # Parse point cloud data
    for i in range(msg.width * msg.height):
        point_data = msg.data[i * msg.point_step:(i + 1) * msg.point_step]
        
        # Extract x, y, z coordinates
        x = np.frombuffer(point_data[0:4], dtype=np.float32)[0]
        y = np.frombuffer(point_data[4:8], dtype=np.float32)[0]
        z = np.frombuffer(point_data[8:12], dtype=np.float32)[0]
        
        # Check if point is valid (not NaN or infinite)
        if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or 
                np.isinf(x) or np.isinf(y) or np.isinf(z)):
            points.append([x, y, z])
            
            # Extract color if available
            if len(point_data) >= 16:  # Assuming RGB is at offset 12-15
                r = np.frombuffer(point_data[12:13], dtype=np.uint8)[0]
                g = np.frombuffer(point_data[13:14], dtype=np.uint8)[0]
                b = np.frombuffer(point_data[14:15], dtype=np.uint8)[0]
                colors.append([r, g, b])
            else:
                colors.append([255, 255, 255])  # Default white color
    
    points = np.array(points)
    colors = np.array(colors) if colors else None
    
    return points, colors

def main():
    parser = argparse.ArgumentParser(description='Extract point clouds from ROS2 bag file to PCD format')
    parser.add_argument('bag_path', help='Path to the ROS2 bag file directory')
    parser.add_argument('--topic', default='/zed/zed_node/mapping/fused_cloud', 
                       help='Topic name containing point cloud data')
    parser.add_argument('--output-dir', default='pcd_exports', 
                       help='Output directory for PCD files')
    
    args = parser.parse_args()
    
    # Check if bag path exists
    if not os.path.exists(args.bag_path):
        print(f"Error: Bag path '{args.bag_path}' does not exist!")
        return
    
    # Extract point clouds
    extract_pointcloud_from_bag(args.bag_path, args.topic, args.output_dir)

if __name__ == '__main__':
    main() 