import numpy as np
import cv2

def radar_polar_to_cartesian(fft_data, azimuths, radar_resolution, cart_resolution=0.2384, cart_pixel_width=640,
                             interpolate_crossover=False, fix_wobble=True):
    # TAKEN FROM PYBOREAS
    """Convert a polar radar scan to cartesian.
    Args:
        azimuths (np.ndarray): Rotation for each polar radar azimuth (radians)
        fft_data (np.ndarray): Polar radar power readings
        radar_resolution (float): Resolution of the polar radar data (metres per pixel)
        cart_resolution (float): Cartesian resolution (metres per pixel)
        cart_pixel_width (int): Width and height of the returned square cartesian output (pixels)
        interpolate_crossover (bool, optional): If true interpolates between the end and start  azimuth of the scan. In
            practice a scan before / after should be used but this prevents nan regions in the return cartesian form.

    Returns:
        np.ndarray: Cartesian radar power readings
    """
    # Compute the range (m) captured by pixels in cartesian scan
    if (cart_pixel_width % 2) == 0:
        cart_min_range = (cart_pixel_width / 2 - 0.5) * cart_resolution
    else:
        cart_min_range = cart_pixel_width // 2 * cart_resolution
    
    # Compute the value of each cartesian pixel, centered at 0
    coords = np.linspace(-cart_min_range, cart_min_range, cart_pixel_width, dtype=np.float32)

    Y, X = np.meshgrid(coords, -1 * coords)
    sample_range = np.sqrt(Y * Y + X * X)
    sample_angle = np.arctan2(Y, X)
    sample_angle += (sample_angle < 0).astype(np.float32) * 2. * np.pi

    # Interpolate Radar Data Coordinates
    azimuth_step = (azimuths[-1] - azimuths[0]) / (azimuths.shape[0] - 1)
    sample_u = (sample_range - radar_resolution / 2) / radar_resolution
    sample_v = (sample_angle - azimuths[0]) / azimuth_step
    # This fixes the wobble in the old CIR204 data from Boreas
    M = azimuths.shape[0]
    azms = azimuths.squeeze()
    if fix_wobble:
        c3 = np.searchsorted(azms, sample_angle.squeeze())
        c3[c3 == M] -= 1
        c2 = c3 - 1
        c2[c2 < 0] += 1
        a3 = azms[c3]
        diff = sample_angle.squeeze() - a3
        a2 = azms[c2]
        delta = diff * (diff < 0) * (c3 > 0) / (a3 - a2 + 1e-14)
        sample_v = (c3 + delta).astype(np.float32)

    # We clip the sample points to the minimum sensor reading range so that we
    # do not have undefined results in the centre of the image. In practice
    # this region is simply undefined.
    sample_u[sample_u < 0] = 0

    if interpolate_crossover:
        fft_data = np.concatenate((fft_data[-1:], fft_data, fft_data[:1]), 0)
        sample_v = sample_v + 1

    polar_to_cart_warp = np.stack((sample_u, sample_v), -1)
    return cv2.remap(fft_data, polar_to_cart_warp, None, cv2.INTER_LINEAR)


'''
def extract_pc(thres_mask, radar_res, azimuth_angles, azimuth_times, vel_input=None, T_ab=None):
    assert(thres_mask.ndim == 3), "thres_mask must be 3D"
    # Threshold the raw scans
    thres_scan = radar_res * np.arange(thres_mask.shape[2]) * thres_mask

    # Find peaks of thresholded points
    peak_points = mean_peaks_parallel_fast(thres_scan)

    # Assemble all points, with azimuth angles and azimuth shapes as the other two dimensions
    azimuth_angles_mat = np.tile( np.expand_dims(azimuth_angles, axis=2), [1, 1, thres_mask.shape[2]] )
    azimuth_times_mat = np.tile( np.expand_dims(azimuth_times, 2), [1, 1, thres_mask.shape[2]] )
    # For azimuth index mat where each row is the azimuth index
    azimuth_index_mat = np.tile( np.expand_dims(np.arange(thres_mask.shape[1]), 1), [1, 1, thres_mask.shape[2]] )
    if vel_input is not None:
        if vel_input.ndim == 2:
            azimuth_vel_mat = np.tile( np.expand_dims(vel_input, 2), [1, 1, thres_mask.shape[2]] )
        else:
            azimuth_vel_mat = vel_input
        peak_pt_mat = np.stack((peak_points, azimuth_angles_mat, azimuth_times_mat, azimuth_vel_mat, azimuth_index_mat), axis=-1)
    else:
        peak_pt_mat = np.stack((peak_points, azimuth_angles_mat, azimuth_times_mat), axis=-1)
    
    # Flatten peak_points along second and third dimensions
    peak_pt_vec = peak_pt_mat.reshape(peak_pt_mat.shape[0], -1, peak_pt_mat.shape[-1])
    pc_list = []

    for ii in range(peak_pt_vec.shape[0]):
        peak_pt_vec_ii = peak_pt_vec[ii]
        nonzero_indices = peak_pt_vec_ii[:,0].nonzero()
        # Find the mean between every two consecutive peaks in nonzero_ii
        nonzero_odd_indices = nonzero_indices[0][1::2]
        nonzero_even_indices = nonzero_indices[0][0::2]
        nonzero_ii_start = peak_pt_vec_ii[nonzero_odd_indices]
        nonzero_ii_end = peak_pt_vec_ii[nonzero_even_indices]

        avg_peak_pt_vec_ii = (nonzero_ii_start + nonzero_ii_end) / 2.0

        pc_ii = pol_2_cart(avg_peak_pt_vec_ii)
        if T_ab is not None:
            T_ii = T_ab[ii]
            pc_ii = (T_ii[:3, :3] @ pc_ii.T).T + T_ii[:3, 3]

        pc_list.append(pc_ii)

    return pc_list


'''




