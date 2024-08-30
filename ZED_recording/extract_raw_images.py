import pyzed.sl as sl
import cv2
import os

def save_raw_frames(svo_file_path, num_frames=100):
    # Create directories for saving images
    left_dir = "left_images"
    right_dir = "right_images"
    os.makedirs(left_dir, exist_ok=True)
    os.makedirs(right_dir, exist_ok=True)

    # Create a Camera object
    zed = sl.Camera()

    # Set initialization parameters
    init_params = sl.InitParameters()
    init_params.set_from_svo_file(svo_file_path)
    #init_params.svo_real_time_mode = False  # Disable real-time mode
    init_params.optional_opencv_calibration_file = "/home/blerim/Otter-data-collection/ZED_recording/SN101391084.conf"

    # Open the SVO file
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open SVO file.")
        return

    # Create Mat objects to hold the images
    left_image = sl.Mat()
    right_image = sl.Mat()

    # Loop to read and save frames
    for i in range(num_frames):
        print("test")
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Retrieve the timestamp
            timestamp = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_milliseconds()

            # Retrieve the left and right images
            zed.retrieve_image(left_image, sl.VIEW.LEFT_UNRECTIFIED)  # Raw left frame
            zed.retrieve_image(right_image, sl.VIEW.RIGHT_UNRECTIFIED)  # Raw right frame

            # Convert to OpenCV format
            left_frame = left_image.get_data()
            right_frame = right_image.get_data()

            # Save the images with the timestamp as the filename
            left_filename = os.path.join(left_dir, f"{timestamp}.png")
            right_filename = os.path.join(right_dir, f"{timestamp}.png")
            cv2.imwrite(left_filename, left_frame)
            cv2.imwrite(right_filename, right_frame)

            print(f"Saved frames {i+1}/{num_frames} with timestamp {timestamp} ms")

        else:
            print(f"Failed to grab frame {i+1}/{num_frames}")
            break

    # Close the camera
    zed.close()

if __name__ == "__main__":
    # Specify the path to your SVO file and the number of frames to save
    svo_file_path = "/media/blerim/capybara2/aug_test_post_lunch.svo2"
    save_raw_frames(svo_file_path, num_frames=100)  # Adjust the number of frames as needed
