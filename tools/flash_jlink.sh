#!/bin/sh
# Download the ELF at its linked addresses through J-Link/SWD.
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
firmware=${1:-"$project_dir/bsp/stm32h743/build/fypilot.elf"}
jlink_exe=${JLINK_EXE:-JLinkExe}
if ! command -v "$jlink_exe" >/dev/null 2>&1; then
    echo "JLinkExe not found. Install SEGGER J-Link Software or set JLINK_EXE." >&2
    exit 127
fi
python3 "$project_dir/tools/verify_firmware.py" "$firmware"
command_file=$(mktemp)
trap 'rm -f "$command_file"' EXIT HUP INT TERM
# The ELF contains the 0x08000000 flash load address; never specify a RAM address.
printf 'r\nh\nloadfile "%s"\nr\ng\nexit\n' "$firmware" > "$command_file"
"$jlink_exe" -device STM32H743VI -if SWD -speed 1000 -AutoConnect 1 \
    -ExitOnError 1 -NoGui 1 -CommandFile "$command_file"
