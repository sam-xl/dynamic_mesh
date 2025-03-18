#! /bin/bash

echo "Run dev mode"
"/ros_entrypoint.sh" &
tail -f /dev/null
