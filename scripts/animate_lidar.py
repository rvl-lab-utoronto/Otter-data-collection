import numpy as np
import open3d as o3d
import os
import cv2
import glob
import json
from tqdm import tqdm

# Define structured dtype for binary data

def load_lidar_bin(file_path):
    """Load LiDAR point cloud from a .bin file."""

    lidar_dtype = np.dtype([
        ("x", np.float64),
        ("y", np.float64),
        ("z", np.float64),
        ("intensity", np.uint16),
        ("timestamp", np.uint64),
        ("reflectivity", np.uint16),
        ("ambient", np.uint16)
    ])
    
    data = np.fromfile(file_path, dtype=lidar_dtype)

    # Extract point cloud and metadata
    points = np.vstack((data['x'], data['y'], data['z'])).T
    intensity = data['intensity'].astype(np.float32)
    
    return points, intensity


def create_timelapse(directory, output_file="timelapse.mp4", fps=100):
    """Creates a timelapse animation from all .bin files in a directory."""
    
    # Get a list of all .bin files and sort them by filename (timestamp order)
    bin_files = sorted(glob.glob(os.path.join(directory, "*.bin")), key=lambda x: int(os.path.basename(x).split(".")[0]))
    
    print(f"Found {len(bin_files)} files for timelapse")
    if not bin_files:
        print("No .bin files found in directory.")
        return
    
    view_json = './lidar_camera_params.json'
    with open(view_json, 'r') as f:
        view = json.load(f)["trajectory"][0]
    
    output_dir = 'lidar_images'
    front = np.array(view["front"])
    lookat = np.array(view["lookat"])
    up = np.array(view["up"])
    zoom = view["zoom"]
    fov_deg = view.get("field_of_view", 60.0)
    fov_rad = np.deg2rad(fov_deg)
    image_width = 800
    image_height = 600

    focal_length = image_width / (2 * np.tan(fov_rad / 2))

    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(
        image_width, image_height,
        focal_length, focal_length,
        image_width / 2, image_height / 2
    )

    eye = lookat - zoom * front

    # Open3D Visualization setup
    render = o3d.visualization.rendering.OffscreenRenderer(image_width, image_height)
    scene = render.scene
    scene.set_background([1,1,1,1]) 
    
    material = o3d.visualization.rendering.MaterialRecord()
    material.shader = "defaultUnlit"

    os.makedirs(output_dir, exist_ok=True)   

    pcd = o3d.geometry.PointCloud()
    for i, file_path in enumerate(tqdm(bin_files)):

        # Load point cloud data from the current file
        points, intensity = load_lidar_bin(file_path)

        # Normalize intensity for coloring
        norm_intensity = (intensity - intensity.min()) / (intensity.max() - intensity.min() + 1e-6)
        colors = np.tile(norm_intensity[:, None], (1, 3))  # RGB color mapping

        # Update point cloud
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        pcd = pcd.voxel_down_sample(voxel_size=0.2)  # 5cm resolution
        
        #bbox = pcd.get_axis_aligned_bounding_box()

        scene.clear_geometry()
        scene.add_geometry("pcd", pcd, material)
        bbox_min = np.array(view["boundingbox_min"], dtype=np.float32)
        bbox_max = np.array(view["boundingbox_max"], dtype=np.float32)
        bbox_center = (bbox_min + bbox_max) / 2
        bbox_extent = bbox_max - bbox_min

        view_direction = np.array([0.0, 0.0, 1.0], dtype=np.float32)

        eye = bbox_center + view_direction * np.linalg.norm(bbox_extent) * 1.5
        up = np.array([0, 1, 0], dtype=np.float32)  # standard up vector

        fov = view.get("field_of_view", 60.0)

        render.setup_camera(
            vertical_field_of_view=fov,
            center=bbox_center,
            eye=eye,
            up=up
        )

        image = render.render_to_image()
        
        out_name = f'{i:05d}.jpg'
        out_path = os.path.join(output_dir, out_name)
        o3d.io.write_image(out_path, image, quality=90)

    print(f"Timelapse saved as {output_file}")

if __name__ == "__main__":
    directory = "/media/blerim/capybara3/aug_22_field_data/lidar"  # Change to the directory containing .bin files
    create_timelapse(directory, output_file="timelapse.mp4", fps=100)
