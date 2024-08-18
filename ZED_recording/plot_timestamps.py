import pyzed.sl as sl
import multiprocessing as mp
import matplotlib.pyplot as plt

def extract_timestamps(chunk):
    svo_file_path, start_frame, end_frame = chunk
    # Create a Camera object
    zed = sl.Camera()

    # Set initialization parameters
    init_params = sl.InitParameters()
    init_params.set_from_svo_file(svo_file_path)
    init_params.svo_real_time_mode = False  # Disable real-time mode

    # Open the SVO file
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        return []

    # Seek to the start frame
    zed.set_svo_position(start_frame)
    
    # List to store timestamps
    timestamps = []
    
    for _ in range(end_frame - start_frame):
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Retrieve the timestamp
            timestamp = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_milliseconds()
            timestamps.append(timestamp)
        else:
            break

    zed.close()
    return timestamps

def plot_timedeltas_parallel(svo_file_path, num_processes=4):
    # Create a Camera object to get the total number of frames
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.set_from_svo_file(svo_file_path)
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open SVO file.")
        return
    total_frames = zed.get_svo_number_of_frames()
    print(total_frames)
    zed.close()

    # Split the frames into chunks for parallel processing
    total_frames = 10
    chunk_size = total_frames // num_processes
    chunks = [(svo_file_path, i, min(i + chunk_size, total_frames)) for i in range(0, total_frames, chunk_size)]

    # Use multiprocessing to extract timestamps in parallel
    with mp.Pool(processes=num_processes) as pool:
        results = pool.map(extract_timestamps, chunks)

    # Flatten the list of lists into a single list of timestamps
    timestamps = [timestamp for sublist in results for timestamp in sublist]

    # Calculate time deltas between consecutive frames
    time_deltas = [j-i for i, j in zip(timestamps[:-1], timestamps[1:])]

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
    svo_file_path = "output.svo2"
    plot_timedeltas_parallel(svo_file_path, num_processes=4)
