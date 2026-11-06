#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
scons -C "$project_dir/bsp/stm32h743" --pyconfig-silent
exec scons -C "$project_dir/bsp/stm32h743" "$@"
