import numpy as np
import cv2

CTS350 = 0    # Oxford
CIR204 = 1    # Boreas

def strToTime(tin):
    tstr = str(tin)
    if '.' in tstr:
        return float(tstr)
    t = float(tstr)
    timeconvert = 1e-6
    if len(tstr) != 16 and len(tstr) > 10:
        timeconvert = 10**(-1 * (len(tstr) - 10))
    return t * timeconvert

def get_time_from_filename(file):
    return strToTime(file.split('.')[0])

def load_lidar(path):
    return np.fromfile(path, dtype=np.float32).reshape((-1, 6))

def load_lidar2(path):
    points = np.fromfile(path, dtype=np.float32).reshape((-1, 6)).astype(np.float64)
    t = get_gps_time_from_file(path)
    points[:, 5] += t
    return points

def load_radar(example_path, fixwob=False):
    """Decode a single Oxford Radar RobotCar Dataset radar example
    Args:
        example_path (AnyStr): Oxford Radar RobotCar Dataset Example png
    Returns:
        timestamps (np.ndarray): Timestamp for each azimuth in int64 (UNIX time)
        azimuths (np.ndarray): Rotation for each polar radar azimuth (radians)
        valid (np.ndarray) Mask of whether azimuth data is an original sensor reading or interpolated from adjacent
            azimuths
        fft_data (np.ndarray): Radar power readings along each azimuth
    """
    # Hard coded configuration to simplify parsing code
    encoder_size = 5600
    raw_example_data = cv2.imread(example_path, cv2.IMREAD_GRAYSCALE)
    if fixwob:
        fix_wobble(raw_example_data)
    timestamps = raw_example_data[:, :8].copy().view(np.int64)
    azimuths = (raw_example_data[:, 8:10].copy().view(np.uint16) / float(encoder_size) * 2 * np.pi).astype(np.float32)
    valid = raw_example_data[:, 10:11] == 255
    fft_data = raw_example_data[:, 11:].astype(np.float32)[:, :, np.newaxis] / 255.
    fft_data = np.squeeze(fft_data)
    return timestamps, azimuths, valid, fft_data

def get_gps_time_from_file(file):
    tstr = file.split('/')[-1].split('.')[0]
    gpstime = float(tstr)
    timeconvert = 1e-9
    if len(tstr) < 19:
        timeconvert = 10**(-1* (len(tstr) - 10))
    return gpstime * timeconvert

def get_azimuth_index(azms, aquery):
    closest = np.argmin(np.abs(azms - aquery))
    if azms[closest] < aquery:
        if closest < azms.shape[0] - 1:
            if azms[closest + 1] == azms[closest]:
                closest += 0.5
            elif azms[closest + 1] > azms[closest]:
                closest += (aquery - azms[closest]) / (azms[closest + 1] - azms[closest])
    elif azms[closest] > aquery:
        if closest > 0:
            if azms[closest - 1] == azms[closest]:
                closest -= 0.5
            elif azms[closest - 1] < azms[closest]:
                closest -= (azms[closest] - aquery) / (azms[closest] - azms[closest - 1])
    return closest

def bilinear_interp(fft_data, aindex):
    ratio = aindex % 1
    if ratio == 0:
        return fft_data[int(aindex)]
    return (1 - ratio) * fft_data[int(np.floor(aindex))] + ratio * fft_data[int(np.ceil(aindex))]

def fix_wobble(raw_data):
    encoder_size = 5600
    azms = (raw_data[:, 8:10].copy().view(np.uint16) / float(encoder_size) * 2 * np.pi).astype(np.float32)
    fft_data = raw_data[:, 11:].copy().astype(np.float32)
    a_step = (azms[-1] - azms[0]) / (azms.shape[0] - 1)
    for i in range(1, azms.shape[0] - 1):
        aquery = azms[0] + a_step * i
        aindex = get_azimuth_index(azms, aquery)
        raw_data[i, 11:] = bilinear_interp(fft_data, aindex).astype(np.uint8)
        raw_data[i, 8:10] = convert_to_byte_array(np.uint16(encoder_size * aquery / (2 * np.pi)), d=16)

def radar_polar_to_cartesian(azimuths, fft_data, radar_resolution, cart_resolution, cart_pixel_width,
                             interpolate_crossover=True, fixwob=True):
    """Convert a polar radar scan to cartesian.
    Args:
        azimuths (np.ndarray): Rotation for each polar radar azimuth (radians)
        fft_data (np.ndarray): Polar radar power readings
        radar_resolution (float): Resolution of the polar radar data (metres per pixel)
        cart_resolution (float): Cartesian resolution (metres per pixel)
        cart_pixel_width (int): Width and height of the returned square cartesian output (pixels). Please see the Notes
            below for a full explanation of how this is used.
        interpolate_crossover (bool, optional): If true interpolates between the end and start  azimuth of the scan. In
            practice a scan before / after should be used but this prevents nan regions in the return cartesian form.

    Returns:
        np.ndarray: Cartesian radar power readings
    """
    if (cart_pixel_width % 2) == 0:
        cart_min_range = (cart_pixel_width / 2 - 0.5) * cart_resolution
    else:
        cart_min_range = cart_pixel_width // 2 * cart_resolution
    coords = np.linspace(-cart_min_range, cart_min_range, cart_pixel_width, dtype=np.float32)
    Y, X = np.meshgrid(coords, -1 * coords)
    sample_range = np.sqrt(Y * Y + X * X)
    sample_angle = np.arctan2(Y, X)
    sample_angle += (sample_angle < 0).astype(np.float32) * 2. * np.pi

    # Interpolate Radar Data Coordinates
    # azimuth_step = azimuths[1] - azimuths[0]
    azimuth_step = (azimuths[-1] - azimuths[0]) / 399
    sample_u = (sample_range - radar_resolution / 2) / radar_resolution
    sample_v = (sample_angle - azimuths[0]) / azimuth_step
    # This fixes the wobble in CIR204 data from Boreas
    EPS = 1e-14
    M = azimuths.shape[0]
    azms = azimuths.squeeze()
    if fixwob:
        c3 = np.searchsorted(azms, sample_angle.squeeze())
        c3[c3 >= M] -= 1
        c2 = c3 - 1
        c2[c2 < 0] += 1
        diff = sample_angle.squeeze() - azms[c3]
        a2 = azms[c2]
        a3 = azms[c3]
        delta = diff * (diff < 0) * (c3 > 0) / (a3 - a2 + EPS)
        sample_v = (c3 + delta).astype(np.float32)

    # We clip the sample points to the minimum sensor reading range so that we
    # do not have undefined results in the centre of the image. In practice
    # this region is simply undefined.
    sample_u[sample_u < 0] = 0

    if interpolate_crossover:
        fft_data = np.concatenate((fft_data[-1:], fft_data, fft_data[:1]), 0)
        sample_v = sample_v + 1

    polar_to_cart_warp = np.stack((sample_u, sample_v), -1)
    return np.expand_dims(cv2.remap(fft_data, polar_to_cart_warp, None, cv2.INTER_LINEAR), axis=0)

# This code is adapted from the Applanix codebase
# X-Y-Z (roll then pitch then yaw) with EXTRINSIC ROTATIONS
# NOTE: they define their principal rotation matrices as the INVERSE of the ones we use at UTIAS
# so the final output needs to be TRANSPOSED to get what we would expect.
# Instead of this, you can simply use our own function: yawPitchRollToRot(y, p, r)
def posOrientToRot(heading, pitch, roll):
    theta = pitch
    phi = roll
    psi = heading

    ctheta = np.cos(theta)
    stheta = np.sin(theta)
    cphi   = np.cos(phi)
    sphi   = np.sin(phi)
    cpsi   = np.cos(psi)
    spsi   = np.sin(psi)

    R = np.identity(3, dtype=np.float64)

    R[0, 0] = ctheta * cpsi
    R[0, 1] = -cphi * spsi + sphi * stheta * cpsi
    R[0, 2] = sphi * spsi + cphi * stheta * cpsi
    R[1, 0] = ctheta * spsi
    R[1, 1] = cphi * cpsi + sphi * stheta * spsi
    R[1, 2] = -sphi * cpsi + cphi * stheta * spsi
    R[2, 0] = -stheta
    R[2, 1] = sphi * ctheta
    R[2, 2] = cphi * ctheta
    return R

# This code is adapted from the Applanix codebase
# Applanix appears to default to soln_num = 1
def applanixRotMatToEulerYPR(R, soln_num=1):
    if np.fabs(R[2, 0]) >= 1:   # singularity
        yaw1 = 0
        yaw2 = 0
        if R[2, 0] < 0:     # gimbal locked down
            delta = np.arctan2(R[0, 1], R[0, 2])
            pitch1 = np.pi/2.0
            pitch2 = np.pi/2.0
            roll1 = delta
            roll2 = delta
        else:   # gimbal locked up
            delta = np.arctan2(-R[0, 1], -R[0, 2])
            pitch1 = -np.pi/2.0
            pitch2 = -np.pi/2.0
            roll1 = delta
            roll2 = delta
    else:
        pitch1 = -np.arcsin(R[2, 0])
        pitch2 = np.pi - pitch1

        roll1 = np.arctan2(R[2, 1] / np.cos(pitch1), R[2, 2] / np.cos(pitch1))
        roll2 = np.arctan2(R[2, 1] / np.cos(pitch2), R[2, 2] / np.cos(pitch2))

        yaw1 = np.arctan2(R[1, 0] / np.cos(pitch1), R[0, 0] / np.cos(pitch1))
        yaw2 = np.arctan2(R[1, 0] / np.cos(pitch2), R[0, 0] / np.cos(pitch2))

    if soln_num == 1:
        return yaw1, pitch1, roll1
    else:
        return yaw2, pitch2, roll2

def roll(r):
    return np.array([[1, 0, 0], [0, np.cos(r), np.sin(r)], [0, -np.sin(r), np.cos(r)]], dtype=np.float64)

def pitch(p):
    return np.array([[np.cos(p), 0, -np.sin(p)], [0, 1, 0], [np.sin(p), 0, np.cos(p)]], dtype=np.float64)

def yaw(y):
    return np.array([[np.cos(y), np.sin(y), 0], [-np.sin(y), np.cos(y), 0], [0, 0, 1]], dtype=np.float64)

def yawPitchRollToRot(y, p, r):
    Y = yaw(y)
    P = pitch(p)
    R = roll(r)
    C = np.matmul(P, Y)
    return np.matmul(R, C)

def rpy2rot(r, p, y):
    Y = yaw(y)
    P = pitch(p)
    R = roll(r)
    C = np.matmul(P, R)
    return np.matmul(Y, C)

def rotToYawPitchRoll(C, eps = 1e-15):
    i = 2
    j = 1
    k = 0
    c_y = np.sqrt(C[i, i]**2 + C[j, i]**2)
    if c_y > eps:
        r = np.arctan2(C[j, i], C[i, i])
        p = np.arctan2(-C[k, i], c_y)
        y = np.arctan2(C[k, j], C[k, k])
    else:
        r = 0
        p = np.arctan2(-C[k, i], c_y)
        y = np.arctan2(-C[j, k], C[j, j])

    return y, p, r

def get_transform(gt):
    """Retrieve 4x4 homogeneous transform for a given parsed line of the ground truth pose csv
    Args:
        gt (List[float]): parsed line from ground truth csv file
    Returns:
        np.ndarray: 4x4 transformation matrix (pose of sensor)
    """
    T = np.identity(4, dtype=np.float64)
    C_enu_sensor = yawPitchRollToRot(gt[9], gt[8], gt[7])
    T[0, 3] = gt[1]
    T[1, 3] = gt[2]
    T[2, 3] = gt[3]
    T[0:3, 0:3] = C_enu_sensor
    return T

def translation_error(T, dim=3):
    """Calculates a euclidean distance corresponding to the translation vector within a 4x4 transform.
    Args:
        T (np.ndarray): 4x4 transformation matrix T = [C, r; 0 0 0 1]
        dim (int): If dim=2 we only use x,y, otherwise we use all dims.
    Returns:
        float: translation distance
    """
    if dim == 2:
        return np.sqrt(T[0, 3]**2 + T[1, 3]**2)
    return np.sqrt(T[0, 3]**2 + T[1, 3]**2 + T[2, 3]**2)

def quaternionToRot(qin):
    q = qin.copy().reshape(4, 1)
    if np.matmul(q.transpose(), q) < 1e-14:
        return np.identity(3)
    xi = q[:3].reshape(3, 1)
    eta = q[3, 0]
    C = (eta**2 - np.matmul(xi.transpose(), xi)) * np.identity(3) + \
        2 * np.matmul(xi, xi.transpose()) - 2 * eta * carrot(xi)
    return C

# Note that the space of unit-length quaternions is a double-cover of SO(3)
# Meaning, C maps to +/- q, so q --> C --> +/- q
def rotToQuaternion(C):
    eta = 0.5 * np.sqrt((1 + np.trace(C)))
    if np.abs(eta) < 1e-14:
        eta = 0
        xi = np.sqrt(np.diag(0.5 * (C + np.identity(3))))
        q = np.array([xi[0], xi[1], xi[2], eta]).reshape(4, 1)
    else:
        phi = wrapTo2Pi(2 * np.arccos(eta))
        eta = np.cos(phi / 2)
        xi_cross = (C.T - C) / (4 * eta)
        q = np.array([xi_cross[2, 1], xi_cross[0, 2], xi_cross[1, 0], eta]).reshape(4, 1)
    return q

def wrapTo2Pi(rads):
    if rads < 0:
        rads += 2 * np.pi
    if rads > 2 * np.pi:
        rads -= 2 * np.pi
    return rads

def rotationError(T):
    d = 0.5 * (np.trace(T[0:3, 0:3]) - 1)
    return np.arccos(max(min(d, 1.0), -1.0))


def get_rotation(heading):
    return np.array([[np.cos(heading), -np.sin(heading), 0],
                     [np.sin(heading), np.cos(heading), 0],
                     [0, 0, 1]])

EARTH_SEMIMAJOR = 6378137.0
EARTH_SEMIMINOR = 6356752.0
EARTH_ECCEN     = 0.081819190842622
EARTH_ECCEN_2 = EARTH_ECCEN*EARTH_ECCEN
a = EARTH_SEMIMAJOR
eccSquared = EARTH_ECCEN**2
eccPrimeSquared = (eccSquared) / (1 - eccSquared)
k0 = 0.9996     # scale factor
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 1.0 / DEG_TO_RAD

def LLtoUTM(latitude, longitude):
    while longitude < -1 * np.pi:
        longitude += 2 * np.pi
    while longitude >= np.pi:
        longitude -= 2 * np.pi
    longDeg = longitude * RAD_TO_DEG
    latDeg = latitude * RAD_TO_DEG
    zoneNumber = int((longDeg + 180) / 6) + 1
    # +3 puts origin in middle of zone
    longOrigin = (zoneNumber - 1) * 6 - 180 + 3
    longOriginRad = longOrigin * DEG_TO_RAD
    N = a / np.sqrt(1 - eccSquared * np.sin(latitude) * np.sin(latitude))
    T = np.tan(latitude) * np.tan(latitude)
    C = eccPrimeSquared * np.cos(latitude) * np.cos(latitude)
    A = np.cos(latitude) * (longitude - longOriginRad)
    M = a * ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 - 5 * eccSquared * eccSquared * eccSquared / 256) * latitude -
        (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32 + 45 * eccSquared * eccSquared * eccSquared / 1024) * np.sin(2 * latitude) +
        (15 * eccSquared * eccSquared / 256 + 45 * eccSquared * eccSquared * eccSquared / 1024) * np.sin(4 * latitude) -
        (35 * eccSquared * eccSquared * eccSquared / 3072) * np.sin(6 * latitude))
    UTMEasting = k0 * N * (A + (1 - T + C) * A * A * A / 6 +
        (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A * A * A * A / 120) + 500000.0
    UTMNorthing = k0 * (M + N * np.tan(latitude) *
        (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
        (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A * A * A * A * A * A / 720))

    if latitude < 0:
        # 10000000 meter offset for southern hemisphere
        UTMNorthing += 10000000.0

    return UTMEasting, UTMNorthing, zoneNumber

def lla2ecef(lla):
    lat = lla[0]
    lon = lla[1]
    alt = lla[2]

    slat = np.sin(lat)
    s2lat = slat*slat
    clat = np.cos(lat)
    slon = np.sin(lon)
    clon = np.cos(lon)
    rprime = EARTH_SEMIMAJOR/np.sqrt(1.0 - EARTH_ECCEN_2*s2lat)

    # position
    x = (rprime + alt)*clat*clon
    y = (rprime + alt)*clat*slon
    z = ((1.0 - EARTH_ECCEN_2)*rprime + alt)*slat

    # rotation
    R_ned_ecef = np.eye(3, dtype=np.float64)
    R_ned_ecef[0, 0] = -slat * clon
    R_ned_ecef[0, 1] = -slat * slon
    R_ned_ecef[0, 2] = clat
    R_ned_ecef[1, 0] = -slon
    R_ned_ecef[1, 1] = clon
    R_ned_ecef[1, 2] = 0
    R_ned_ecef[2, 0] = -clat * clon
    R_ned_ecef[2, 1] = -clat * slon
    R_ned_ecef[2, 2] = -slat

    # output transform
    T_ecef_ned = np.eye(4, dtype=np.float64)
    T_ecef_ned[0, 3] = x
    T_ecef_ned[1, 3] = y
    T_ecef_ned[2, 3] = z
    T_ecef_ned[:3, :3] = R_ned_ecef.T

    return T_ecef_ned

def RelLLAtoNED(lla, ll0):
    # make sure ll0 shape is 3
    assert(ll0.ndim == 1)
    assert(ll0.shape[0] == 3)

    # make sure lla shape is 3
    assert(lla.ndim == 1)
    assert(lla.shape[0] == 3)

    # positions in ECEF frame
    T_ecef_ned = lla2ecef(lla)
    T_ecef_nedref = lla2ecef(ll0)

    # output
    T_nedref_ned = np.matmul(get_inverse_tf(T_ecef_nedref), T_ecef_ned)
    return T_nedref_ned

def undistort(img, K, dist, roi=None, P=None):
    dst = cv2.undistort(img, K, dist, None, P)
    if roi is not None and P is not None:
        h, w, _ = img.shape
        x, y, w2, h2 = roi
        dst = dst[y:y+h2, x:x+w2]
        dst = cv2.resize(dst, (w, h))
    return dst

# d can be 8, 16, 32, 64, ...
def convert_to_byte_array(t, d=64):
    byte_array = np.zeros(d, dtype=np.uint8)
    for i in range(d-1, -1, -1):
        k = t >> i
        if (k & 1):
            byte_array[i] = 1
        else:
            byte_array[i] = 0
    out_array = np.zeros(d // 8, dtype=np.uint8)
    for i in range(d // 8):
        for j in range(8):
            out_array[i] += byte_array[i * 8 + j] * 2**j
    return out_array

def get_inverse_tf(T):
    """Returns the inverse of a given 4x4 homogeneous transform.
    Args:
        T (np.ndarray): 4x4 transformation matrix
    Returns:
        np.ndarray: inv(T)
    """
    T2 = np.identity(4, dtype=T.dtype)
    R = T[0:3, 0:3]
    t = T[0:3, 3].reshape(3, 1)
    T2[0:3, 0:3] = R.transpose()
    T2[0:3, 3:] = np.matmul(-1 * R.transpose(), t)
    return T2

def carrot(xbar):
    """Overloaded operator. converts 3x1 vectors into a member of Lie Alebra so(3)
        Also, converts 6x1 vectors into a member of Lie Algebra se(3)
    Args:
        xbar (np.ndarray): if 3x1, xbar is a vector of rotation angles, if 6x1 a vector of 3 trans and 3 rot angles.
    Returns:
        np.ndarray: Lie Algebra 3x3 matrix so(3) if input 3x1, 4x4 matrix se(3) if input 6x1.
    """
    x = xbar.squeeze()
    if x.shape[0] == 3:
        return np.array([[0, -x[2], x[1]],
                         [x[2], 0, -x[0]],
                         [-x[1], x[0], 0]])
    elif x.shape[0] == 6:
        return np.array([[0, -x[5], x[4], x[0]],
                         [x[5], 0, -x[3], x[1]],
                         [-x[4], x[3], 0, x[2]],
                         [0, 0, 0, 1]])
    print('WARNING: attempted carrot operator on invalid vector shape')
    return xbar

def se3ToSE3(xi):
    """Converts 6x1 vectors representing the Lie Algebra, se(3) into a 4x4 homogeneous transform in SE(3)
        Lie Vector xi = [rho, phi]^T (6 x 1) --> SE(3) T = [C, r; 0 0 0 1] (4 x 4)
    Args:
        xi (np.ndarray): 6x1 vector
    Returns:
        np.ndarray: 4x4 transformation matrix
    """
    T = np.identity(4, dtype=np.float32)
    rho = xi[0:3].reshape(3, 1)
    phibar = xi[3:6].reshape(3, 1)
    phi = np.linalg.norm(phibar)
    R = np.identity(3)
    if phi != 0:
        phibar /= phi  # normalize
        I = np.identity(3)
        R = np.cos(phi) * I + (1 - np.cos(phi)) * np.matmul(phibar, phibar.T) + np.sin(phi) * carrot(phibar)
        J = I * np.sin(phi) / phi + (1 - np.sin(phi) / phi) * np.matmul(phibar, phibar.T) + \
            carrot(phibar) * (1 - np.cos(phi)) / phi
        rho = np.matmul(J, rho)
    T[0:3, 0:3] = R
    T[0:3, 3:] = rho
    return T


def se3_curly_wedge(xi):
    # Returns Nx4x4 tensor with each 1x6 row vector in xi SE(3) curly wedge'd
    Xi = np.zeros((xi.shape[0], 6, 6), dtype=xi.dtype)
    rho = xi[:, 0:3]
    phi = xi[:, 3:6]
    Phi = so3_wedge(phi)
    Rho = so3_wedge(rho)
    Xi[:, 0:3, 0:3] = Phi
    Xi[:, 0:3, 3:6] = Rho
    Xi[:, 3:6, 3:6] = Phi
    return Xi

def so3_wedge(phi):
    # Returns Nx3x3 tensor with each 1x3 row vector in phi wedge'd
    Phi = np.zeros((phi.shape[0], 3, 3), dtype=phi.dtype)
    Phi[:, 0, 1] = -phi[:, 2]
    Phi[:, 1, 0] = phi[:, 2]
    Phi[:, 0, 2] = phi[:, 1]
    Phi[:, 2, 0] = -phi[:, 1]
    Phi[:, 1, 2] = -phi[:, 0]
    Phi[:, 2, 1] = phi[:, 0]
    return Phi