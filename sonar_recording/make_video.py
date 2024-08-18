import numpy as np
import matplotlib.pyplot as plt
import argparse
import matplotlib.animation as animation
from oculus_python.files import OculusFileReader

def update_plot(frame, messages, ax1, ax2, ax3, ax4):
    bearings, linearAngles, gains, rawPingData, pingData, timestamp = messages[frame]

    # Clear and plot new data
    ax1.clear()
    ax1.plot(bearings, '-o', label='bearings')
    ax1.plot(linearAngles, '-o', label='linear bearings')
    ax1.grid()
    ax1.legend()
    ax1.set_xlabel('bearing index')
    ax1.set_ylabel('bearing angle')
    ax1.set_title(f'Timestamp: {timestamp}')

    ax2.clear()
    ax2.plot(gains, '-o', label='gains')
    ax2.grid()
    ax2.legend()
    ax2.set_xlabel('range index')
    ax2.set_ylabel('range gain')
    ax2.set_title('Gains')

    ax3.clear()
    ax3.imshow(rawPingData, aspect='auto')
    ax3.set_ylabel('Range index')
    ax3.set_xlabel('Bearing index')
    ax3.set_title('Raw Ping Data')

    ax4.clear()
    ax4.imshow(pingData, aspect='auto')
    ax4.set_xlabel('Bearing index')
    ax4.set_title('Rescaled Ping Data')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='OculusFileReader',
        description='Create a video from .oculus file data.')
    parser.add_argument('filename', type=str,
                        help='Path to a .oculus file to display')
    args = parser.parse_args()

    print('Opening', args.filename)

    f = OculusFileReader(args.filename)
    messages = []
    timestamps = []
    msg = f.read_next_ping()

    while msg is not None:
        # Extract all necessary data
        bearings = 0.01 * np.array(msg.bearing_data())
        linearAngles = np.linspace(bearings[0], bearings[-1], len(bearings))
        gains = np.ones([msg.range_count(),], dtype=np.float32)
        if msg.has_gains():
            gains = np.array(msg.gains())
        rawPingData = np.array(msg.raw_ping_data())
        pingData = np.array(msg.ping_data()) / np.sqrt(gains)[:, np.newaxis]
        timestamp = msg.timestamp_micros() * 1e-6

        # Convert timestamp to seconds (assuming timestamp is a datetime object)
        #timestamp_seconds = timestamp.total_seconds() if hasattr(timestamp, 'total_seconds') else timestamp

        # Store the extracted data and timestamp in seconds
        messages.append((bearings, linearAngles, gains, rawPingData, pingData, timestamp))
        timestamps.append(timestamp)

        msg = f.read_next_ping()

    if not messages:
        print('File seems to be empty. Aborting.')
        exit()

    # Calculate time differences between consecutive frames in seconds
    time_diffs = np.diff(timestamps)
    print(time_diffs)
    mean_time_diff = np.mean(time_diffs) if len(time_diffs) > 0 else 1.0

    # Calculate fps based on the mean time difference
    fps = 1.0 / mean_time_diff if mean_time_diff > 0 else 1.0
    print(f'Calculated FPS based on timestamps: {fps:.2f}')

    # Set up the figure and axes for the animation
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)

    # Create the animation
    ani = animation.FuncAnimation(fig, update_plot, frames=len(messages),
                                  fargs=(messages, ax1, ax2, ax3, ax4), repeat=False)

    # Save the animation as an mp4 file
    video_filename = 'output_video.mp4'
    #ani.save(video_filename, writer='ffmpeg', fps=fps)

    ani.save(video_filename, writer='ffmpeg', fps=fps, extra_args=['-vcodec', 'libx264', '-preset', 'fast'])
    print(f'Video saved as {video_filename} with FPS: {fps:.2f}')
