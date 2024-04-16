#!/bin/bash

# Check if FFmpeg is installed
if ! command -v ffmpeg &> /dev/null; then
    echo "Error: FFmpeg is not installed. Please install FFmpeg first."
    exit 1
fi

# Check if correct number of arguments are provided
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <input_directory> <frame_rate> <output_file>"
    exit 1
fi

# Assign command-line arguments to variables
input_dir="$1"
frame_rate="$2"
output_file="$3"

# Check if input directory exists
if [ ! -d "$input_dir" ]; then
    echo "Error: Input directory '$input_dir' not found."
    exit 1
fi

# Check if input directory contains JPG files
if ! ls "$input_dir"/*.jpg &> /dev/null; then
    echo "Error: No JPG files found in input directory '$input_dir'."
    exit 1
fi

# Convert JPG images to MP4 using FFmpeg
ffmpeg -framerate "$frame_rate" -i "$input_dir"/image%d.jpg "$output_file"

echo "Conversion completed. MP4 file created: $output_file"
