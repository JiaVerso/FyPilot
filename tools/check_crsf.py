#!/usr/bin/env python3
"""Check the portable CRSF parser with upstream vectors and ASan/UBSan."""
import os
import subprocess
import tempfile
from pathlib import Path

root = Path(__file__).resolve().parents[1]
rx = root / 'app/rx'
with tempfile.TemporaryDirectory(prefix='fypilot-crsf-') as directory:
    program = Path(directory) / 'crsf-check'
    subprocess.run([os.environ.get('HOST_CC', 'cc'), '-std=c99', '-Wall', '-Wextra',
                    '-Werror', '-O1', '-g', '-fsanitize=address,undefined',
                    '-fno-omit-frame-pointer', '-I' + str(rx),
                    str(root / 'tools/check_crsf.c'), str(rx / 'crsf_parser.c'),
                    '-o', str(program)], check=True)
    subprocess.run([str(program)], check=True)
