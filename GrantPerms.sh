#!/bin/bash

# Yes, this was made with ChatGPT

if [[ $EUID -ne 0 ]]; then
  echo -e "\033[1;33mERROR: This script must be run with sudo or as the root user.\033[0m"
  exit 1
fi

echo -e "\033[1;33mWARNING: This script will give ALL permissions (read, write, execute) to ALL files and folders in the current directory and its subdirectories.\033[0m"
read -p "Are you sure you want to proceed? (yes/no): " confirmation

if [[ $confirmation != "yes" ]]; then
  echo "Operation canceled by the user."
  exit 0
fi

echo "Granting permissions..."
for item in $(find .); do
  chmod 777 "$item"
  echo "Permissions granted for: $item"
done

echo "All permissions have been granted, now rock."



