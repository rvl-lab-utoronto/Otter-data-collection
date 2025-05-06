import os
import numpy as np
import pandas as pd
from utils import se3_curly_wedge, se3ToSE3

def adjoint(T):
    # Returns Nx6x6 adjoint for Nx4x4 input SE(3) transforms
    if T.ndim < 3:
        T = T[None]
    Tad = np.zeros((T.shape[0], 6, 6), dtype=T.dtype)
    C = T[:, :3, :3]
    Jrho = T[:, :3, -1]
    Tad[:, :3, :3] = C
    Tad[:, 3:, 3:] = C
    Tad[:, :3, 3:] = so3_wedge(Jrho) @ C
    return Tad

def get_inverse_tf_batch(T):
    """Returns the inverses of a given Nx4x4 batch of homogeneous transforms.
    Args:
        T (np.ndarray): Nx4x4 transformation matrices
    Returns:
        np.ndarray: Nx4x4 inv(T)
    """
    if T.ndim < 3:
        T = T[None]
    Rt = T[:, 0:3, 0:3].transpose(0, 2, 1)
    t = T[:, 0:3, 3:4]
    T_inv = T.copy()
    T_inv[:, 0:3, 0:3] = Rt
    T_inv[:, 0:3, 3:4] = -Rt @ t
    return T_inv

def roll_batch(x):
    C = np.zeros((x.shape[0], 3, 3), dtype=x.dtype)
    C[:, 0, 0] = 1
    C[:, 1, 1] = np.cos(x)
    C[:, 1, 2] = np.sin(x)
    C[:, 2, 1] = -np.sin(x)
    C[:, 2, 2] = np.cos(x)
    return C

def pitch_batch(x):
    C = np.zeros((x.shape[0], 3, 3), dtype=x.dtype)
    C[:, 0, 0] = np.cos(x)
    C[:, 0, 2] = -np.sin(x)
    C[:, 1, 1] = 1
    C[:, 2, 0] = np.sin(x)
    C[:, 2, 2] = np.cos(x)
    return C

def yaw_batch(x):
    C = np.zeros((x.shape[0], 3, 3), dtype=x.dtype)
    C[:, 0, 0] = np.cos(x)
    C[:, 0, 1] = np.sin(x)
    C[:, 1, 0] = -np.sin(x)
    C[:, 1, 1] = np.cos(x)
    C[:, 2, 2] = 1
    return C

def yawPitchRollToRotBatch(y, p, r):
    return roll_batch(r) @ pitch_batch(p) @ yaw_batch(y)

class ICPEstimates:
    """Load icp odometry estimates"""
    def __init__(self, path, seq):
        # modify this class to load your odometry results
        # need body-centric velocity estimates and timestamps

        # pose file
        path_to_pose = os.path.join(path, seq + '_poses.txt')
        df = pd.read_csv(path_to_pose, header=None, delimiter=' ')
        temp = df.to_numpy()

        self.Tis = np.repeat(np.eye(4)[None], repeats=temp.shape[0], axis=0)
        self.Tis[:, :3] = temp.reshape((-1, 3, 4))

        # velocity file
        path_to_vel = os.path.join(path, seq + '_vels.txt')
        df = pd.read_csv(path_to_vel, header=None, delimiter=' ')
        temp = df.to_numpy()

        # load times
        self.times = temp[:, 1] * 1e-6
        # self.ftimes = temp[:, 0] * 1e-6

        # load body-centric velocities (sensor frame)
        self.varpi = temp[:, 2:]

class ApplanixGT:
    """Load applanix post processed estimates"""
    def __init__(self, path, dtype):
        self.dtype = dtype
        pdcsv = pd.read_csv(os.path.join(path, 'applanix/gps_post_process.csv'))
        self.times = pdcsv.loc[:, 'GPSTime'].to_numpy()
        self.Tia = self.pdcsv_to_se3_pose(pdcsv)
        self.varpi = self.pdcsv_to_body_vel(pdcsv, self.Tia)
    
    def pdcsv_to_se3_pose(self, pdcsv):
        Tis = np.zeros((pdcsv.shape[0], 4, 4), dtype=self.dtype)
        Tis[:, -1, -1] = 1
        Tis[:, :3, :3] = yawPitchRollToRotBatch(pdcsv.loc[:, 'heading'], pdcsv.loc[:, 'pitch'], pdcsv.loc[:, 'roll'])
        Tis[:, 0, -1] = pdcsv.loc[:, 'easting']
        Tis[:, 1, -1] = pdcsv.loc[:, 'northing']
        Tis[:, 2, -1] = pdcsv.loc[:, 'altitude']
        return Tis

    def pdcsv_to_body_vel(self, pdcsv, Tis):
        w = np.zeros((pdcsv.shape[0], 3), dtype=self.dtype)    # angular (already in sensor frame)
        v = np.zeros((pdcsv.shape[0], 3), dtype=self.dtype)    # translational    (in inertial frame)
        w[:, 0] = pdcsv.loc[:, 'angvel_x']
        w[:, 1] = pdcsv.loc[:, 'angvel_y']
        w[:, 2] = pdcsv.loc[:, 'angvel_z']
        v[:, 0] = pdcsv.loc[:, 'vel_east']
        v[:, 1] = pdcsv.loc[:, 'vel_north']
        v[:, 2] = pdcsv.loc[:, 'vel_up']
        v = Tis[:, :3, :3].transpose(0, 2, 1) @ v[:, :, None] 
        return -np.concatenate((v[:, :, 0], w), axis=1)

# linear interpolation used for velocity
def linear_interp(src_times, src_varpi, tgt_times):
    tail = 0
    head = 1
    _tgt_varpi = []
    for ts in tgt_times:
        while not src_times[tail] <= ts <= src_times[head]:
            tail += 1
            head += 1
        assert(src_times[tail] <= ts)
        assert(src_times[head] >= ts)
        alpha = (ts - src_times[tail])/(src_times[head] - src_times[tail])  # (1 - a)*k1 + a*k2
        _tgt_varpi += [(1-alpha)*src_varpi[tail] + alpha*src_varpi[head]]
    return np.array(_tgt_varpi, dtype=src_varpi.dtype)

if __name__ == '__main__':
    # Optimize extrinsic calibration between lidar and applanix frames.
    # Similar method can be used for other sensors, like radar.
    #
    # We optimize the extrinsic transform between the body-centric velocity estimates, so we need the odometry
    # velocity estimates and their corresponding timestamps (assumed to be in the sensor frame).
    # For applanix, we load the velocity from 'gps_post_process.csv' and linearly interpolate for sensor times
    #
    # Estimating all 6 DOF is difficult. Locking some of the dimensions is recommended (particularly z-translation).
    # For lidar, we lock xyz translation, roll, and pitch, and only estimate 1 DOF (yaw)

    # params
    max_iter = 20   # 20 seems to be enough, but manually check that the cost converges
    icp_root = '/home/asrl/ASRL/steam_icp/temp/boreas_velodyne/steam/vel_write/'   # path to odom estimates. Modify ICPEstimates class to load your odom estimates
    data_root = '/home/asrl/ASRL/steam_icp/data/boreas-aeries/'  # path to boreas dataset. Will load 'gps_post_process.csv' from 'applanix' directory
    seq_names = ['2023_02_15_1', '2023_02_15_3', '2023_02_15_5']    # sequences you want to use to optimize (need their odometry estimates)
    dtype = np.float64

    # initialize the extrinsic calib
    # note: if you're planning on locking roll and pitch to zero, make sure your initialization has them as 0
    T_applanix_sensor_init = np.loadtxt(os.path.join(data_root, seq_names[0], 'calib', 'T_applanix_lidar.txt'))

    # active_dims controls which dimensions we allow the optimizer to change.
    # e.g., active_dims = np.array([3, 4, 5], dtype=np.int64) will optimize roll, pitch, yaw
    # e.g., active_dims = np.array([5], dtype=np.int64) will only optimize yaw
    active_dims = np.array([5], dtype=np.int64)

    times_list = []
    applnx_list = []
    icp_list = []
    for seq_name in seq_names:
        # load applanix groundtruth
        applnx = ApplanixGT(os.path.join(data_root, seq_name), dtype)
    
        # load icp estimates
        icp = ICPEstimates(os.path.join(icp_root, seq_name), seq_name)

        # interpolate applanix velocities for odometry times
        assert(icp.times[0] >= applnx.times[0])
        assert(icp.times[-1] <= applnx.times[-1])
        interp_varpi = linear_interp(applnx.times, applnx.varpi, icp.times)
        applnx_list += [interp_varpi]
        icp_list += [icp.varpi]
        times_list += [icp.times]

    # initialize T_applanix_sensor
    Tas = T_applanix_sensor_init.copy()

    # dim params
    P = np.eye(6, dtype=dtype)[:, active_dims]

    # iterations
    cost_list = []
    for iter in range(max_iter):
        lhs = np.zeros((active_dims.shape[0], active_dims.shape[0]), dtype=dtype)
        rhs = np.zeros((active_dims.shape[0], 1), dtype=dtype)
        lazy_cost = 0
        for icp, applnx in zip(icp_list, applnx_list):
            adTw = (adjoint(Tas) @ icp[:, :, None])[:, :, 0]
            ebar = applnx - adTw    # error
            jac = se3_curly_wedge(adTw) @ P[None]   # jacobian
            lazy_cost += np.sum(ebar * ebar).item()

            lhs += np.sum(jac.transpose(0, 2, 1) @ jac, axis=0)
            rhs += np.sum(-jac.transpose(0, 2, 1) @ ebar[:, :, None], axis=0)

        # solve
        dx = np.linalg.solve(lhs, rhs)
        cost_list += [lazy_cost]

        # update
        dxi = np.zeros(6, dtype=dtype)
        dxi[active_dims] = dx.squeeze(-1)
        Tas = se3ToSE3(dxi) @ Tas

    print('T_applanix_sensor init:')
    print(T_applanix_sensor_init)
    print('T_applanix_sensor:')
    print(Tas)

    print(cost_list)    # should plot and double check that it converges
