#!/bin/bash

# Directory containing images
IMG_DIR="/media/blerim/capybara2/aug_22_field_data/sonar/"
# Output video filename
OUTPUT="timelapse.mp4"
# Desired framerate of the timelapse
FRAMERATE=30

# Create a temporary file list in the correct order
find "$IMG_DIR" -type f -name "*.png" | sort > frames.txt

# Generate the timelapse using FFmpeg with NVENC (H.264)
ffmpeg -y \
  -f concat \
  -safe 0 \
  -r "$FRAMERATE" \
  -i <(awk '{print "file \x27" $0 "\x27"}' frames.txt) \
  -c:v h264_nvenc \
  -preset fast \
  -pix_fmt yuv420p \
  "$OUTPUT"

# Clean up
rm frames.txt

