#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
rsync -avz --progress --exclude '*.egg-info .git .gitignore' -e 'ssh -p 22' "$SCRIPT_DIR/../" hayashi-rpi@imaging.local:/home/hayashi-rpi/imaging_driver