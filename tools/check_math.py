#!/usr/bin/env python3
"""Build native scalar CMSIS-DSP tests; no target hardware or RTOS emulation."""
import os
import subprocess
import tempfile
from pathlib import Path
root = Path(__file__).resolve().parents[1]
math_dir = root / 'app/common/math'
dsp = root / 'sdk/cmsis-dsp'
with tempfile.TemporaryDirectory(prefix='fypilot-math-') as directory:
    program = Path(directory) / 'math-check'
    command = [os.environ.get('HOST_CC', 'cc'), '-std=c99', '-O2', '-Wall', '-Wextra',
               '-D__GNUC_PYTHON__', '-DARM_MATH_MATRIX_CHECK',
               '-I' + str(math_dir), '-I' + str(dsp / 'Include'),
               str(root / 'tools/check_math.c'), str(math_dir / 'my_math.c')]
    command += [str(path) for path in sorted((dsp / 'Source/MatrixFunctions').glob('*.c'))]
    subprocess.run(command + ['-lm', '-o', str(program)], check=True)
    subprocess.run([str(program)], check=True)
