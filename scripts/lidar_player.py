
import open3d as o3d
import numpy as np
import os
import time
import threading

def load_lidar_bin(file_path):
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
    points = np.vstack((data['x'], data['y'], data['z'])).T
    intensity = data['intensity'].astype(np.float32)
    
    return points, intensity

class InteractiveLidarPlayer:
    def __init__(self, directory, ext=".bin", fps=10):
        self.files = sorted([
            os.path.join(directory, f) for f in os.listdir(directory)
            if f.endswith(ext)
        ])
        self.fps = fps
        self.current_idx = 0
        self.running = True
        self.quit_flag = False

        self.pcd = o3d.geometry.PointCloud()
        self.load_frame(0)

        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window("LiDAR Player", width=1280, height=720)
        self.vis.add_geometry(self.pcd)

        # Register key callbacks
        self.vis.register_key_callback(ord(" "), self.toggle_play)
        self.vis.register_key_callback(ord("Q"), self.quit)

    def load_frame(self, index):
        points, intensity = load_lidar_bin(self.files[index])
        norm_intensity = intensity / (np.max(intensity) + 1e-6)
        colors = np.tile(norm_intensity[:, None], (1, 3))

        self.pcd.clear()
        self.pcd.points = o3d.utility.Vector3dVector(points)
        self.pcd.colors = o3d.utility.Vector3dVector(colors)

    def toggle_play(self, vis):
        self.running = not self.running
        print("[Paused]" if not self.running else "[Playing]")
        return False

    def quit(self, vis):
        print("Quitting...")
        self.quit_flag = True
        return False

            
    def run(self):
        for i in range(20000, len(self.files)):
            self.load_frame(i)
            self.vis.update_geometry(self.pcd)
            self.vis.poll_events()
            self.vis.update_renderer()
        self.vis.run()
        self.vis.destroy_window()
# === Run ===
if __name__ == "__main__":
    player = InteractiveLidarPlayer("/home/robot/data/aug_22_field_data/lidar/", fps=10)
    player.run()
