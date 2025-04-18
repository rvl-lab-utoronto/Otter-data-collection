import numpy as np
import open3d as o3d

# Define structured dtype based on provided field sizes
lidar_dtype = np.dtype([
    ("x", np.float64),
    ("y", np.float64),
    ("z", np.float64),
    ("intensity", np.uint16),  # 16-bit unsigned int
    ("timestamp", np.uint64),  # 64-bit timestamp
    ("reflectivity", np.uint16),
    ("ambient", np.uint16)
])

def load_lidar_bin(file_path):
    """Load LiDAR point cloud with mixed field sizes from .bin file."""
    data = np.fromfile(file_path, dtype=lidar_dtype)

    points = np.vstack((data['x'], data['y'], data['z'])).T  # Shape (N, 3)
    intensity = data['intensity'].astype(np.float32)  # Convert to float for visualization

    return points, intensity

def visualize_point_cloud(points, intensity):
    """Visualize LiDAR point cloud with intensity coloring."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Normalize intensity for coloring
    intensity = (intensity - intensity.min()) / (intensity.max() - intensity.min())
    colors = np.tile(intensity[:, None], (1, 3))  # Apply to RGB

    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    o3d.visualization.draw_geometries([pcd], window_name="LiDAR Point Cloud")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="LiDAR Point Cloud Visualizer")
    parser.add_argument("file", type=str, help="Path to the LiDAR .bin file")
    args = parser.parse_args()
    
    points, intensity = load_lidar_bin(args.file)
    visualize_point_cloud(points, intensity)

