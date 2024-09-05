import pyzed.sl as sl
import cv2
import os

def save_raw_frames(svo_file_path, num_frames=100):
    # Create directories for saving images
    left_dir = root+"left_images"
    right_dir = root+"right_images"
    depth_dir = "depth_images"
    os.makedirs(left_dir, exist_ok=False)
    os.makedirs(right_dir, exist_ok=False)
    os.makedirs(depth_dir, exist_ok=True)

    # Create a Camera object

    # Set initialization parameters
    init_params = sl.InitParameters()
    init_params.set_from_svo_file(svo_file_path)
    init_params.svo_real_time_mode = False  # Disable real-time mode
    init_params.coordinate_units = sl.UNIT.MILLIMETER
    #init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    #init_params.optional_opencv_calibration_file = "/home/blerim/Otter-data-collection/ZED_recording/SN101391084.conf"

    zed = sl.Camera()
    # Open the SVO file
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open SVO file.")
        return

    # Create Mat objects to hold the images
    left_image = sl.Mat()
    right_image = sl.Mat()
    depth_image = sl.Mat()

    # Loop to read and save frames
    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Retrieve the timestamp
            timestamp = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_microseconds()

            # Retrieve the left and right images
            zed.retrieve_image(left_image, sl.VIEW.LEFT_UNRECTIFIED)  # Raw left frame
            zed.retrieve_image(right_image, sl.VIEW.RIGHT_UNRECTIFIED)  # Raw right frame
            #zed.retrieve_image(depth_image, sl.VIEW.DEPTH)

            # Convert to OpenCV format
            left_frame = left_image.get_data()
            right_frame = right_image.get_data()
            #depth_frame = depth_image.get_data()

            # Save the images with the timestamp as the filename
            left_filename = os.path.join(root, left_dir, f"{timestamp}.png")
            right_filename = os.path.join(root, right_dir, f"{timestamp}.png")
            #depth_filename = os.path.join(root, depth_dir, f"{timestamp}.png")
            cv2.imwrite(left_filename, left_frame)
            cv2.imwrite(right_filename, right_frame)
            #cv2.imwrite(depth_filename, depth_frame)

            print(f"Saved frame pair with timestamp {timestamp} ms")

        else:
            print(f"Failed to grab frame {i+1}/{num_frames}")
            break

    # Close the camera
    zed.close()

if __name__ == "__main__":
    # Specify the path to your SVO file and the number of frames to save
    svo_file_path = "/media/blerim/capybara2/aug_23_field_data/aug_23_run_2_launch.svo2"
    root = "/media/diskstation/otter-2024-08-23-12/camera/"
    os.mkdir(root)
    save_raw_frames(svo_file_path)  # Adjust the number of frames as needed
