import rosbag2_py
import cv2
import numpy as np
import open3d as o3d
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import rclpy.serialization
import os

def save_image_and_pointcloud(image_msg, pointcloud_msg, index):
    # Deserialize the image message
    image = rclpy.serialization.deserialize_message(image_msg, Image)
    
    # Convert ROS Image message to OpenCV image
    img_array = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
    image_cv = img_array#cv2.cvtColor(img_array, cv2.COLOR)
    
    # Save image as PNG
    image_filename = f"calib_files/images/{index:05}.png"
    cv2.imwrite(image_filename, image_cv)
    print(f"Saved image: {image_filename}")
    
    # Deserialize the point cloud message
    pointcloud = rclpy.serialization.deserialize_message(pointcloud_msg, PointCloud2)

    # Convert ROS PointCloud2 message to a list of points and then to a regular NumPy array
    points = np.array(list(point_cloud2.read_points(pointcloud, field_names=("x", "y", "z"), skip_nans=True)))

    # Extract just the x, y, z values and convert to a float32 array
    points_xyz = np.vstack((points['x'], points['y'], points['z'])).T.astype(np.float32)
    
    # Create an Open3D PointCloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz)
    
    # Save point cloud as PCD
    pcd_filename = f"calib_files/scans/{index:05}.pcd"
    o3d.io.write_point_cloud(pcd_filename, pcd)
    print(f"Saved pointcloud: {pcd_filename}")

def find_closest_pairs(bag_file_path, max_pairs=400):
    # Initialize ROS2 bag reader
    bag = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    bag.open(storage_options, converter_options)
    print("test")

    image_msgs = []
    pointcloud_msgs = []

    # Iterate through the bag and separate the messages
    count = 0
    while bag.has_next():
        topic, data, timestamp = bag.read_next()
        count += 1
        if count < 300:
            continue
        if len(pointcloud_msgs) > 800:
            break
        if topic == "/zed/zed_node/right_raw/image_raw_color":
            image_msgs.append((timestamp, data))
        elif topic == "/ouster/points":
            pointcloud_msgs.append((timestamp, data))
    
    #skip_first = 500
    #image_msgs = image_msgs[skip_first:]
    #pointcloud_msgs = pointcloud_msgs[int(skip_first/3):]
    # Find the closest pairs
    pairs = []
    for pc_timestamp, pc_msg in pointcloud_msgs:
        closest_image_msg = min(image_msgs, key=lambda x: abs(x[0] - pc_timestamp))
        pairs.append((closest_image_msg[1], pc_msg))

        # Stop if we have enough pairs
        if len(pairs) >= max_pairs:
            break

    # Save the closest pairs
    for i, (image_msg, pointcloud_msg) in enumerate(pairs):
        save_image_and_pointcloud(image_msg, pointcloud_msg, i)

# Path to your ROS2 bag file
bag_file_path = "/home/robot/data/9_ml_lake_calib/rosbag2_2024_08_22-22_09_56"

# Extract and save the closest pairs
os.makedirs('./calib_files/images/', exist_ok=True)
os.makedirs('./calib_files/scans/', exist_ok=True)
find_closest_pairs(bag_file_path)
