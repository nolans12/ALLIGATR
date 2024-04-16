#!/bin/bash

# Function to check if FFmpeg is installed
check_ffmpeg() {
    if ! command -v ffmpeg &> /dev/null; then
        echo "Error: FFmpeg is not installed. Please install FFmpeg first."
        exit 1
    fi
}

# Function to convert JPG images to MP4
convert_to_mp4() {
    input_dir="$1"
    frame_rate="$2"

    # Get output filename
    output_file="$input_dir/video.mp4"

    # Convert JPG images to MP4 using FFmpeg
    ffmpeg -framerate "$frame_rate" -i "$input_dir"/image%d.jpg "$output_file"

    echo "Conversion completed. MP4 file created: $output_file"
}

# Main function
main() {
    # Check if FFmpeg is installed
    check_ffmpeg

    # Check if correct number of arguments are provided
    if [ "$#" -ne 2 ]; then
        echo "Usage: $0 <frame_rate> <folder>"
        exit 1
    fi

    frame_rate="$1"
    folder="$2"

    # Check if folder exists
    if [ ! -d "$folder" ]; then
        echo "Error: Folder '$folder' not found."
        exit 1
    fi

    # Check if folder contains JPG files
    if ! ls "$folder"/*.jpg &> /dev/null; then
        echo "Error: No JPG files found in folder '$folder'."
        exit 1
    fi

    # Convert JPG images to MP4
    convert_to_mp4 "$folder" "$frame_rate"
}

# Run the main function
main "$@"
