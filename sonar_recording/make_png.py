import numpy as np
import matplotlib.pyplot as plt
import argparse
import matplotlib.animation as animation
from PIL import Image
from oculus_python.files import OculusFileReader
from datetime import datetime, timezone
import os

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

def update_single_plot(msg, ax1, ax2, ax3, ax4):
    bearings, linearAngles, gains, rawPingData, pingData, timestamp = msg

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

def print_fields(msg):
    print(msg.metadata())
    print(msg.ping_index())
    print(msg.range())
    print(msg.gain_percent())
    print(msg.frequency())
    print(msg.speed_of_sound_used())
    print(msg.range_resolution())
    print(msg.temperature())
    print(msg.pressure())
    print(msg.bearing_data())
    print(msg.timestamp_micros())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='OculusFileReader',
        description='Create a video from .oculus file data.')
    parser.add_argument('--filename', type=str,
                        help='Path to a .oculus file to display')

    parser.add_argument('--outdir', type=str,
                        help='output dir')
    args = parser.parse_args()
    outdir = args.outdir

    print('Opening', args.filename)
    counter = 0

    f = OculusFileReader(args.filename)
    messages = []
    timestamps = []
    msg = f.read_next_ping()
    os.makedirs(outdir, exist_ok=True)
    while msg is not None:
        timestamp = msg.timestamp_micros() #* 1e-6
        timestamps.append(timestamp)
        print(datetime.fromtimestamp(timestamp*1e-6, tz=timezone.utc))

        if counter < 0: # skip the part on land, for now, save everything
            msg = f.read_next_ping()
            counter += 1
            continue
        # elif counter > 30000: # take first 1/3 part the the data
        #     break
        else: 
            # Extract all necessary data
            bearings = 0.01 * np.array(msg.bearing_data())
            linearAngles = np.linspace(bearings[0], bearings[-1], len(bearings))
            gains = np.ones([msg.range_count(),], dtype=np.float32)
            if msg.has_gains():
                gains = np.array(msg.gains())
            rawPingData = np.array(msg.raw_ping_data())
            pingData = np.array(msg.ping_data()) / np.sqrt(gains)[:, np.newaxis]

            msg_data = (bearings, linearAngles, gains, rawPingData, pingData, timestamp)

            # save fig every x frames, you can change this
            if counter % 1 == 0:
                length, width = rawPingData.shape
                # normalized_data = ((rawPingData - rawPingData.min()) / (rawPingData.max() - rawPingData.min()) * 255).astype(np.uint8)
                # print("is normalized data equal to rawPingData?", np.array_equal(normalized_data, rawPingData))
                image = Image.fromarray(rawPingData)
                image.save(outdir + f"/{timestamp}.png")

                ## print the size of rawPingData
                # print("rawPingData shape:", rawPingData.shape)
                # print("pingData shape:", pingData.shape)
                # print("bearings shape:", bearings.shape)
                # print("gains shape:", gains.shape)
                # print("linearAngles shape:", linearAngles.shape)

                ## double check if the image is saved correctly
                # recovered_image = Image.open(f"output/images/{timestamp}.png")
                # recovered_image_data = np.array(recovered_image)
                # print(np.array_equal(rawPingData, recovered_image_data))

                # save a text file with all the other meta info in msg
                
                #with open(f"output/txt/{timestamp}.txt", "w") as t:
                #    t.write(f"timestamp: {timestamp}\n")
                #    t.write(f"ping_index: {msg.ping_index()}\n")
                #    t.write(f"range: {msg.range()}\n")
                #    t.write(f"gain_percent: {msg.gain_percent()}\n")
                #    t.write(f"frequency: {msg.frequency()}\n")
                #    t.write(f"speed_of_sound_used: {msg.speed_of_sound_used()}\n")
                #    t.write(f"range_resolution: {msg.range_resolution()}\n")
                #    t.write(f"temperature: {msg.temperature()}\n")
                #    t.write(f"pressure: {msg.pressure()}\n")
                #    t.write(f"bearing_data: {bearings}\n")
                #    t.write(f"linear_angles: {linearAngles}\n")
                #    # np.savetxt(t, rawPingData, delimiter=",", fmt="%d")

                #    t.close()
    
        counter += 1
        msg = f.read_next_ping()

    # Calculate time differences between consecutive frames in seconds
    time_diffs = np.diff(timestamps)
    print(time_diffs)

    # plot a histogram of the time differences, display bins between 0 and 0.5 seconds
    plt.hist(time_diffs, bins=np.arange(0, 0.3, 0.01))
    plt.xlabel('Time difference (s)')
    plt.ylabel('Frequency')
    plt.title('Histogram of time differences between consecutive frames')
    plt.savefig("output/histogram_of_time_diff.png")
    plt.close()

    mean_time_diff = np.mean(time_diffs) if len(time_diffs) > 0 else 1.0

    # Calculate fps based on the mean time difference
    fps = 1.0 / mean_time_diff if mean_time_diff > 0 else 1.0
    print(f'Calculated FPS based on timestamps: {fps:.2f}')

    # # Set up the figure and axes for the animation
    # fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)

    # # Create the animation
    # ani = animation.FuncAnimation(fig, update_plot, frames=len(messages),
    #                               fargs=(messages, ax1, ax2, ax3, ax4), repeat=False)

    # # Save the animation as an mp4 file
    # video_filename = 'output_video.mp4'
    # #ani.save(video_filename, writer='ffmpeg', fps=fps)

    # ani.save(video_filename, writer='ffmpeg', fps=fps, extra_args=['-vcodec', 'libx264', '-preset', 'fast'])
    # print(f'Video saved as {video_filename} with FPS: {fps:.2f}')
