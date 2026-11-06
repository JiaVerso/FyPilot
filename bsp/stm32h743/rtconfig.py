"""Portable GCC toolchain settings; no external RT-Thread checkout required."""
import glob
import os
import shutil
ARCH = 'arm'
CPU = 'cortex-m7'
CROSS_TOOL = 'gcc'
PLATFORM = 'gcc'
BSP_LIBRARY_TYPE = None
PREFIX = 'arm-none-eabi-'
compiler = shutil.which(PREFIX + 'gcc')
candidates = sorted(glob.glob('/opt/st/stm32cubeclt_*/GNU-tools-for-STM32/bin'), reverse=True)
EXEC_PATH = os.environ.get('RTT_EXEC_PATH') or (os.path.dirname(compiler) if compiler else (candidates[0] if candidates else ''))
BUILD = os.environ.get('BUILD', 'release')
CC = AS = LINK = PREFIX + 'gcc'
CXX = PREFIX + 'g++'
AR = PREFIX + 'ar'
SIZE = PREFIX + 'size'
OBJCPY = PREFIX + 'objcopy'
OBJDUMP = PREFIX + 'objdump'
TARGET_EXT = 'elf'
DEVICE = '-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard'
BASE = DEVICE + ' -ffunction-sections -fdata-sections -Dgcc -g3 '
BASE += '-Og' if BUILD == 'debug' else '-O2'
CFLAGS = BASE + ' -std=c99 -Wall -Wno-unused-parameter'
CXXFLAGS = BASE + ' -std=c++14 -fno-exceptions -fno-rtti'
AFLAGS = DEVICE + ' -c -x assembler-with-cpp -g3'
LFLAGS = DEVICE + ' -nostartfiles --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections,-Map=build/fypilot.map,-cref -T board/linker_scripts/link.lds -Wl,--print-memory-usage'
CPATH = LPATH = ''
POST_ACTION = OBJCPY + ' -O binary $TARGET build/fypilot.bin\n' + OBJCPY + ' -O ihex $TARGET build/fypilot.hex\n' + SIZE + ' $TARGET'
