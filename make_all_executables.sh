#!/bin/bash

# Check if a directory was passed as an argument, otherwise use the current directory
DIRECTORY="${1:-.}"

# Print the directory being scanned
echo "Scanning directory: $DIRECTORY"

# Find all files in the directory (and subdirectories), exclude directories, and make them executable
find "$DIRECTORY" -type d | while read -r dir; do
  echo "Scanning directory: $dir"
  # Find files in this directory and mark them as executable
  find "$dir" -maxdepth 1 -type f -exec chmod +x {} \; -exec echo "Signed and made executable: {}" \;
done

echo "All files in $DIRECTORY and its subdirectories are now executable."

