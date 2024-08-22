import open3d as o3d

# Path to your PCD file
pcd_file_path = "./calib_files/scans/pointcloud_11.pcd"

# Load the PCD file
pcd = o3d.io.read_point_cloud(pcd_file_path)

# Print some information about the PCD file
print(pcd)

# Visualize the PCD file
o3d.visualization.draw_geometries([pcd])
