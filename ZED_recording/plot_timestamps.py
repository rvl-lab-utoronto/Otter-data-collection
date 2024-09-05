import pyzed.sl as sl
import multiprocessing as mp
import matplotlib.pyplot as plt
import numpy as np



def plot_timedeltas_dir(image_dir):
    
    left_images = sorted([img for img in os.listdir(left_images_dir) if img.endswith(('png', 'jpg', 'jpeg'))])
    right_images = sorted([img for img in os.listdir(right_images_dir) if img.endswith(('png', 'jpg', 'jpeg'))])
    
    if len(left_images) != len(right_images):
        print("The number of images in both directories must be the same.")
        exit()


def plot_timedeltas(svo_file_path):
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.set_from_svo_file(svo_file_path)
    init_params.svo_real_time_mode = False  # Disable real-time mode
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open SVO file.")
        return
    total_frames = zed.get_svo_number_of_frames()
    timestamps = [] 
    left_image = sl.Mat()
    right_image = sl.Mat()
    num_frames = 10000
    count = 0
    while count < num_frames:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Retrieve the timestamp

            #zed.retrieve_image(left_image, sl.VIEW.LEFT_UNRECTIFIED)
            #zed.retrieve_image(right_image, sl.VIEW.RIGHT_UNRECTIFIED)  
            
            timestamp_left = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_microseconds()
            timestamp_right = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_microseconds()
            timestamps.append(timestamp_left)
            #print(str(timestamp_left) + "  |   " +  str(timestamp_right)) 
            #left_filename = os.path.join(root, left_dir, f"{timestamp}.png")
            #right_filename = os.path.join(root, right_dir, f"{timestamp}.png")
            #depth_filename = os.path.join(root, depth_dir, f"{timestamp}.png")
            
            #cv2.imwrite(left_filename, left_frame)
            #cv2.imwrite(right_filename, right_frame)
 
            #cv2.imwrite(depth_filename, depth_frame)
            count += 1

        else:
            print(f"Failed to grab frame {i+1}/{num_frames}")
            break
        
    timestamps = np.array(timestamps)
    time_deltas = np.diff(timestamps)
    print(np.max(time_deltas))
    # Plot the time deltas
    plt.figure(figsize=(10, 6))
    plt.plot(time_deltas, marker='o', linestyle='-', color='b')
    plt.title("Time Deltas Between Consecutive Left Frames")
    plt.xlabel("Frame Index")
    plt.ylabel("Time Delta (ms)")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    # Specify the path to your SVO file and the number of processes
    svo_file_path = "./aug_test_post_lunch.svo2"
    plot_timedeltas(svo_file_path)
