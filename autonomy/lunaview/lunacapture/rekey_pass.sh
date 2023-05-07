#!/bin/sh
# Create a new cryptographically random password. 
cat /dev/urandom | tr -cd a-zA-Z0-9_ | head -c 20 > .dbpass
echo "New pass created."

