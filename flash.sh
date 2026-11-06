#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if command -v "${JLINK_EXE:-JLinkExe}" >/dev/null 2>&1; then
    exec "$project_dir/tools/flash_jlink.sh" "$@"
fi
if command -v openocd >/dev/null 2>&1; then
    exec "$project_dir/tools/flash_openocd.sh" "$@"
fi
echo "Install SEGGER J-Link Software (JLinkExe) or OpenOCD with J-Link support." >&2
exit 127
