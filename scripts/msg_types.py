
PACKETMSG = """
uint8[] buf
"""

FFT_MSG = """
# A ROS message based on an FFT data message from a radar, with corrected types

# add a header message to hold message timestamp
std_msgs/Header header

# angle (float64) represented as a network order (uint8) byte array
float64 angle

# azimuth (uint16) represented as a network order (uint8) byte array
uint16 azimuth

# sweep_counter (uint16) represented as a network order (uint8) byte array
uint16 sweep_counter

# ntp_seconds (uint32) represented as a network order (uint8) byte array
uint32 ntp_seconds

# ntp_split_seconds (uint32) represented as a network order (uint8) byte array
uint32 ntp_split_seconds

# data (uint8) represented as a network order (uint8) byte array
uint8[] data

# data_length (uint16) represented as a network order (uint8) byte array
uint16 data_length"""

CFG_MSG = """
# A ROS message based on a configuration data message from a radar, with corrected types

# add a header message to hold message timestamp
std_msgs/Header header

# azimuth_samples (uint16)
uint16 azimuth_samples

# encoder_size (uint16)
uint16 encoder_size

# user-provided azimuth offset (uint16)
uint16 azimuth_offset

# bin_size (float64)
float32 bin_size

# range_in_bins (uint16)
uint16 range_in_bins

# expected_rotation_rate (uint16)
uint16 expected_rotation_rate

# range_gain (float32)
float32 range_gain

# range_offset (float32)
float32 range_offset"""

BSCAN_MSG = """
# B Scan from one rotation of the radar, also holds the time stamp information
sensor_msgs/Image b_scan_img

# The encoder values encompassed by the b scan
uint16[] encoder_values

# The timestamps of each azimuth in the scan
uint64[] timestamps"""
