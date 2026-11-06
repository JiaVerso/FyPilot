#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
firmware=${1:-"$project_dir/bsp/stm32h743/build/fypilot.elf"}
python3 "$project_dir/tools/verify_firmware.py" "$firmware"
# Quote the absolute filename as a Tcl braced value, preserving spaces.
escaped_firmware=$(printf '%s' "$firmware" | sed 's/[\\{}]/\\&/g')
exec openocd -f "$project_dir/tools/jlink-swd.cfg" \
    -c "program {$escaped_firmware} verify reset exit"
