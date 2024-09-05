import pyzed.sl as sl
import cv2

def preview_svo_raw_frames(svo_file_path, num_frames=100):
    # Create a Camera object
    zed = sl.Camera()

    # Set initialization parameters
    init_params = sl.InitParameters()
    init_params.set_from_svo_file(svo_file_path)
    init_params.svo_real_time_mode = False  # Disable real-time mode

    # Open the SVO file
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open SVO file.")
        return

    # Create Mat objects to hold the images
    left_image = sl.Mat()
    right_image = sl.Mat()

    # Loop to read and display frames
    for i in range(num_frames):
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Retrieve the left and right images
            zed.retrieve_image(left_image, sl.VIEW.LEFT_UNRECTIFIED)  # Raw left frame
            zed.retrieve_image(right_image, sl.VIEW.RIGHT_UNRECTIFIED)  # Raw right frame

            # Convert to OpenCV format
            left_frame = left_image.get_data()
            right_frame = right_image.get_data()

            # Display the frames side by side
            combined_frame = cv2.hconcat([left_frame, right_frame])
            cv2.imshow("Raw Left and Right Frames", combined_frame)

            # Exit if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print(f"Failed to grab frame {i+1}/{num_frames}")
            break

    # Close the camera
    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Specify the path to your SVO file and the number of frames to preview
    svo_file_path = "./aug_test_post_lunch.svo2"
    preview_svo_raw_frames(svo_file_path, num_frames=100)  # Adjust the number of frames as needed
