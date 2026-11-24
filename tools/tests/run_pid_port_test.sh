#!/usr/bin/env bash
set -eu
repo_dir=$(cd -- "$(dirname -- "$0")/../.." && pwd)
test_bin=$(mktemp /tmp/fypilot-pid-test.XXXXXX)
trap 'rm -f "$test_bin"' EXIT
"${CC:-cc}" -std=c99 -Wall -Wextra -Werror -g \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$repo_dir/applications" -I"$repo_dir/applications/core/pid" \
    "$repo_dir/applications/core/pid/pid.c" \
    "$repo_dir/applications/core/pid/pid_init.c" \
    "$repo_dir/applications/common/filter.c" \
    "$repo_dir/tools/tests/pid_port_test.c" -lm -o "$test_bin"
"$test_bin"
