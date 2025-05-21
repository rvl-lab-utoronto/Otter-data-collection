import argparse
import csv
import os
import sys
import pandas as pd
from calendar import c
import numpy as np
import matplotlib.pyplot as plt
from pyproj import Transformer

# Make sure your ROS 2 workspace is sourced so Python can find custom message modules
try:
    from vtr_navigation_msgs.msg import RobotState
    from novatel_oem7_msgs.msg import BESTPOS
except ImportError:
    sys.exit("Error: Could not import 'vtr_navigation_msgs.msg.RobotState'. Make sure you have sourced your workspace and that the package is in your PYTHONPATH.")

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message


def extract_data(bag_path: str):
    # Fixed topic for robot state
    robotstate_topic_name = '/vtr/robot_state'
    gnss_topic_name = '/novatel/oem7/bestpos'

    # Prepare reader for the rosbag2 bag
    storage_options = StorageOptions(
        uri=bag_path,
        storage_id='sqlite3'
    )
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Determine message type for the topic
    topics_info = reader.get_all_topics_and_types()
    found = False
    for info in topics_info:
        if info.name == robotstate_topic_name:
            found = True
            break
    if not found:
        raise ValueError(f"Topic '{robotstate_topic_name}' not found in bag")

    # Two dataframes, one for each topic
    robotstate_records = []
    gnss_records = []

    times_odom = []
    times_gnss = []

    # Read through the bag and extract lat/lng
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == robotstate_topic_name:
            msg = deserialize_message(data, RobotState)
            lat = getattr(msg, 'lat', None)
            lng = getattr(msg, 'lng', None)
            robotstate_records.append({'lat': lat, 'lng': lng})
            times_odom.append(timestamp)
        elif topic == gnss_topic_name:
            msg = deserialize_message(data, BESTPOS)
            lat = getattr(msg, 'lat', None)
            lng = getattr(msg, 'lon', None)
            gnss_records.append({'lat': lat, 'lng': lng})
            times_gnss.append(timestamp)

    # Convert to DataFrames
    robotstate_df = pd.DataFrame(robotstate_records)
    gnss_df = pd.DataFrame(gnss_records)
    return times_odom, times_gnss, robotstate_df, gnss_df


# Geodetic to ENU conversion via ECEF intermediary
def geodetic_to_enu(latlng_arr, lat0, lon0, h0=0):
    transformer = Transformer.from_crs("epsg:4979", "epsg:4978", always_xy=True)
    x0, y0, z0 = transformer.transform(lon0, lat0, h0)
    xyz = np.array([transformer.transform(lon, lat, h0) for lat, lon in latlng_arr])
    d = xyz - np.array([x0, y0, z0])
    sin_lat = np.sin(np.deg2rad(lat0)); cos_lat = np.cos(np.deg2rad(lat0))
    sin_lon = np.sin(np.deg2rad(lon0)); cos_lon = np.cos(np.deg2rad(lon0))
    enu = np.zeros_like(d)
    enu[:,0] = -sin_lon*d[:,0] + cos_lon*d[:,1]
    enu[:,1] = -cos_lon*sin_lat*d[:,0] - sin_lon*sin_lat*d[:,1] + cos_lat*d[:,2]
    enu[:,2] =  cos_lat*cos_lon*d[:,0] + cos_lat*sin_lon*d[:,1] + sin_lat*d[:,2]
    return enu

# ICP-like alignment (using first 100 points)
def icp(A, B, max_iter=20, tol=1e-6):
    """
    Align B to A using an iterative closest point approach on 2D (E,N).
    Returns rotation matrix R and translation t.
    """
    # Initialize transform
    R_total = np.eye(2)
    t_total = np.zeros(2)
    B_trans = B.copy()

    for i in range(max_iter):
        # Find nearest neighbors in A for each point in B_trans
        dists = np.sqrt(((B_trans[:, None, :] - A[None, :, :])**2).sum(axis=2))
        idx = np.argmin(dists, axis=1)
        A_matched = A[idx]
        # Compute centroids
        mu_A = A_matched.mean(axis=0)
        mu_B = B_trans.mean(axis=0)
        # Center
        AA = A_matched - mu_A
        BB = B_trans - mu_B
        # SVD for best rotation
        H = BB.T.dot(AA)
        U, S, Vt = np.linalg.svd(H)
        R_iter = Vt.T.dot(U.T)
        if np.linalg.det(R_iter) < 0:
            Vt[-1,:] *= -1
            R_iter = Vt.T.dot(U.T)
        t_iter = mu_A - R_iter.dot(mu_B)
        # Update total transform
        R_total = R_iter.dot(R_total)
        t_total = R_iter.dot(t_total) + t_iter
        # Apply to B
        B_trans = (R_iter.dot(B_trans.T)).T + t_iter
        # Check convergence
        if np.linalg.norm(t_iter) < tol and np.linalg.norm(R_iter - np.eye(2)) < tol:
            break
    return R_total, t_total

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Extract latitude and longitude from /vtr/robot_state in a ROS2 Humble bag and write to CSV.'
    )
    parser.add_argument('bag_path', help='Path to the ROS2 bag directory')
    args = parser.parse_args()

    if not os.path.isdir(args.bag_path):
        print(f"Error: Bag path '{args.bag_path}' does not exist or is not a directory.")
        exit(1)

    times_odom, times_gnss, robotstate_df, gnss_df = extract_data(args.bag_path)
    
    lat0_1, lon0_1 = gnss_df.iloc[0][['lat', 'lng']]
    lat0_2, lon0_2 = robotstate_df.iloc[0][['lat', 'lng']]

    enu1 = geodetic_to_enu(gnss_df[['lat', 'lng']].values, lat0_1, lon0_1)
    enu2 = geodetic_to_enu(robotstate_df[['lat', 'lng']].values, lat0_2, lon0_2)

    # Use only East, North for ICP
    A2 = enu1[500:900, :2]
    B2 = enu2[500:900, :2]

    R2, t2 = icp(A2, B2)
    # Apply to full track
    B_all = enu2[:, :2]
    B_aligned = (R2.dot(B_all.T)).T

    print("R2:\n", R2)
    print("t2:\n", t2)

    # ------------------------------------------------------------------------
    # 1) Build time‐synchronized trajectories
    # ------------------------------------------------------------------------
    # Convert timestamp lists to numpy
    odom_ts = np.array(times_odom)   # ns
    gps_ts  = np.array(times_gnss)   # ns

    max_dt = 1e9  # 1 second tolerance
    end_timestamp_seconds = 1747411646

    gt_pts = []
    est_pts = []
    j = 0
    for i, t_g in enumerate(gps_ts):
        if t_g > end_timestamp_seconds * 1e9:
            break
        # advance j to the odometry sample closest to this GPS time
        while (j+1 < len(odom_ts) and
               abs(odom_ts[j+1] - t_g) <= abs(odom_ts[j] - t_g)):
            j += 1
        if abs(odom_ts[j] - t_g) <= max_dt:
            gt_pts.append( enu1[i, :2] )
            est_pts.append( B_aligned[j] )
    gt_pts  = np.array(gt_pts)
    est_pts = np.array(est_pts)

    print(f"Number of synchronized samples: {len(gt_pts)}")
    print(f"Number of odometry samples: {len(enu2)}")
    print(f"Number of GPS samples: {len(enu1)}")

    if len(gt_pts) < 2:
        raise RuntimeError("Not enough synchronized samples within tolerance")

    # ------------------------------------------------------------------------
    # 2) Compute cumulative ground‐truth distance along the synced GPS track
    # ------------------------------------------------------------------------
    disp = np.linalg.norm(np.diff(gt_pts, axis=0), axis=1)
    cumulative_dist = np.hstack(([0.], np.cumsum(disp)))  # length M

    # ------------------------------------------------------------------------
    # 3) KITTI translational drift on synced tracks
    # ------------------------------------------------------------------------
    segment_lengths = np.arange(100, 900, 100)
    t_err_per_L = []

    for L in segment_lengths:
        errs = []
        # for each start index i
        for i in range(len(cumulative_dist)):
            target = cumulative_dist[i] + L
            j = np.searchsorted(cumulative_dist, target, side='left')
            if j >= len(cumulative_dist):
                break
            delta_gt  = gt_pts[j]  - gt_pts[i]
            delta_est = est_pts[j] - est_pts[i]
            drift = np.linalg.norm(delta_est - delta_gt) * 100.0 / L
            errs.append(drift)
            # print(f"Drift @ {L} m: {np.linalg.norm(delta_est - delta_gt):.3f} m")

        t_err_per_L.append(np.mean(errs) if errs else np.nan)

    # ------------------------------------------------------------------------
    # 4) Report
    # ------------------------------------------------------------------------
    for L, err in zip(segment_lengths, t_err_per_L):
        print(f"Translational drift @ {L} m: {err:.3f} %")
    mean_drift = np.nanmean(t_err_per_L)
    print(f"Mean translational drift (100-800 m): {mean_drift:.3f} %")

    # Plot
    plt.figure()
    plt.plot(gt_pts[:,0], gt_pts[:,1], label='Ground Truth (GPS)')
    plt.plot(est_pts[:,0], est_pts[:,1], label='VT&R LiDAR Odometry')
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.axis('equal')
    plt.legend()
    plt.title('Odometry vs Ground Truth')
    plt.show()

