import cv2
import argparse
import os

def speed_up_video(input_path, output_path, speed_factor):
    # Open the video file
    cap = cv2.VideoCapture(input_path)
    if not cap.isOpened():
        print(f"Error: Could not open video file {input_path}")
        return

    # Get original video properties
    original_fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Set new FPS to speed up the video
    new_fps = original_fps * speed_factor

    # Initialize VideoWriter with new FPS
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, new_fps, (width, height))

    # Read and write frames
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        out.write(frame)

    # Release resources
    cap.release()
    out.release()
    print(f"Video saved: {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Speed up multiple MP4 videos.")
    parser.add_argument("videos", nargs='+', help="List of input MP4 files.")
    parser.add_argument("--speed_factor", type=float, default=2.0, help="Factor by which to speed up the video.")
    parser.add_argument("--output_folder", type=str, default="output", help="Folder to save the sped-up videos.")

    args = parser.parse_args()

    # Ensure output folder exists
    if not os.path.exists(args.output_folder):
        os.makedirs(args.output_folder)

    # Process each video
    for video_path in args.videos:
        base_name = os.path.basename(video_path)
        output_path = os.path.join(args.output_folder, f"spedup_{base_name}")
        speed_up_video(video_path, output_path, args.speed_factor)
