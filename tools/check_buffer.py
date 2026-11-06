#!/usr/bin/env python3
"""Native buffer logic check; real RT-Thread mutex/ISR timing needs board tests."""
import os
import subprocess
import tempfile
from pathlib import Path
root = Path(__file__).resolve().parents[1]
buffer_dir = root / 'app/common/buffer'
with tempfile.TemporaryDirectory(prefix='fypilot-buffer-') as directory:
    directory = Path(directory)
    (directory / 'rtthread.h').write_text('''#include <stdint.h>
#include <stddef.h>
typedef int rt_err_t;
typedef int rt_bool_t;
struct rt_mutex { int initialized; };
#define RT_NULL NULL
#define RT_TRUE 1
#define RT_FALSE 0
#define RT_EOK 0
#define RT_EINVAL 22
#define RT_EBUSY 16
#define RT_WAITING_FOREVER -1
#define RT_IPC_FLAG_PRIO 1
static inline int rt_mutex_init(struct rt_mutex *m, const char *n, int f)
{ (void)n; (void)f; m->initialized=1; return 0; }
static inline int rt_mutex_take(struct rt_mutex *m, int t)
{ (void)t; return m->initialized ? 0 : -22; }
static inline int rt_mutex_release(struct rt_mutex *m)
{ return m->initialized ? 0 : -22; }
static inline int rt_mutex_detach(struct rt_mutex *m)
{ m->initialized=0; return 0; }
''')
    program = directory / 'buffer-check'
    subprocess.run([os.environ.get('HOST_CC', 'cc'), '-std=c99', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-g', '-I' + str(directory),
                    '-I' + str(buffer_dir), str(root / 'tools/check_buffer.c'),
                    str(buffer_dir / 'buffer.c'), '-o', str(program)], check=True)
    subprocess.run([str(program)], check=True)
