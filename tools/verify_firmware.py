#!/usr/bin/env python3
"""Check an STM32H743VI ELF and its vector table without loading hardware."""
import argparse
import struct
from pathlib import Path

FLASH_BASE, FLASH_END = 0x08000000, 0x08200000
RAM_BASE, RAM_END = 0x24000000, 0x24080000


def check(path: Path, profile: str = 'auto', require_crsf: bool = False) -> dict:
    image = path.read_bytes()
    if len(image) < 52 or image[:7] != b'\x7fELF\x01\x01\x01':
        raise ValueError('Expected a little-endian ELF32 executable')
    (_, kind, machine, _, entry, phoff, shoff, flags, _, phsize, phnum,
     shsize, shnum, _) = struct.unpack_from('<16sHHIIIIIHHHHHH', image)
    if kind != 2 or machine != 40 or not flags & 0x400:
        raise ValueError('Expected an ARM executable with the hard-float ABI')
    flash_segments = []
    ram_end = RAM_BASE
    for i in range(phnum):
        ptype, offset, vaddr, paddr, filesz, memsz, _, _ = struct.unpack_from(
            '<8I', image, phoff + i * phsize)
        if ptype != 1:
            continue
        if FLASH_BASE <= paddr < FLASH_END and filesz:
            if paddr + filesz > FLASH_END or offset + filesz > len(image):
                raise ValueError('Flash load segment exceeds the device or file')
            flash_segments.append((paddr, image[offset:offset + filesz]))
        if RAM_BASE <= vaddr < RAM_END:
            if vaddr + memsz > RAM_END:
                raise ValueError('RAM segment exceeds the AXI SRAM region')
            ram_end = max(ram_end, vaddr + memsz)
        elif not (FLASH_BASE <= vaddr < FLASH_END):
            raise ValueError(f'Unexpected load address 0x{vaddr:08x}')
    first = next((data for addr, data in flash_segments if addr == FLASH_BASE), None)
    if first is None or len(first) < 8:
        raise ValueError('Missing flash vector table at 0x08000000')
    stack, reset = struct.unpack_from('<II', first)
    if not (RAM_BASE < stack <= RAM_END) or stack % 8:
        raise ValueError(f'Invalid initial stack pointer 0x{stack:08x}')
    if not reset & 1 or not entry & 1 or (entry & ~1) != (reset & ~1):
        raise ValueError('ELF entry and Thumb reset vector do not agree')
    if not any(addr <= (reset & ~1) < addr + len(data) for addr, data in flash_segments):
        raise ValueError('Reset handler is outside the flash image')
    # Verify the boot path and the selected application are actually linked.
    sections = [struct.unpack_from('<10I', image, shoff + i * shsize) for i in range(shnum)]
    symbols = set()
    symbol_values = {}
    for section in sections:
        _, stype, _, _, offset, size, link, _, _, entsize = section
        if stype != 2:
            continue
        string_section = sections[link]
        strings = image[string_section[4]:string_section[4] + string_section[5]]
        for pos in range(offset, offset + size, entsize):
            name, value, _, _, _, defined = struct.unpack_from('<IIIBBH', image, pos)
            if name and defined:
                symbol = strings[name:strings.find(b'\x00', name)].decode('utf-8', errors='replace')
                symbols.add(symbol)
                symbol_values[symbol] = value
    has_hello = any(b'Hello World\n\x00' in data for _, data in flash_segments)
    if profile == 'auto':
        profile = 'hello' if has_hello else 'imu'
    required = {'Reset_Handler', 'entry', 'rt_system_scheduler_start', 'SysTick_Handler',
                'rt_hw_usart_init', 'main'}
    if require_crsf or 'crsf_receiver_start' in symbols:
        # crsf_receiver_get may be inlined into the shell command by GCC.
        required |= {'crsf_receiver_start', 'crsf_parser_feed', 'crsf',
                     'crsf_receiver_entry', 'USART6_IRQHandler'}
    if profile == 'hello':
        if not has_hello:
            raise ValueError('Hello World message is missing from the flash image')
        if {'aopa_bmi270_init', 'imu_entry'} & symbols:
            raise ValueError('Hello World profile unexpectedly includes the IMU application')
    else:
        required |= {'rt_hw_spi_init', 'aopa_bmi270_init', 'bmi270_init',
                     'imu_entry', 'mp_selftest'}
    missing = required - symbols
    if missing:
        raise ValueError('Missing target functions: ' + ', '.join(sorted(missing)))
    if require_crsf or 'crsf_receiver_start' in symbols:
        # STM32H743 CMSIS: USART6_IRQn = 71. Reject the startup weak alias.
        handler = symbol_values['USART6_IRQHandler']
        if handler == symbol_values.get('Default_Handler') or len(first) < 4 * (16 + 72):
            raise ValueError('USART6 interrupt has no real implementation/vector')
        if struct.unpack_from('<I', first, 4 * (16 + 71))[0] != (handler | 1):
            raise ValueError('USART6 vector does not point to the UART driver')
    flash_end = max(addr + len(data) for addr, data in flash_segments)
    binary = bytearray(b'\xff' * (flash_end - FLASH_BASE))
    for addr, data in flash_segments:
        binary[addr - FLASH_BASE:addr - FLASH_BASE + len(data)] = data
    bin_path = path.with_suffix('.bin')
    if bin_path.exists() and bin_path.read_bytes() != binary:
        raise ValueError('BIN does not match ELF flash load segments')
    hex_path = path.with_suffix('.hex')
    if hex_path.exists():
        memory = {}
        base = 0
        eof = False
        for line in hex_path.read_text().splitlines():
            if not line.startswith(':'):
                raise ValueError('Malformed HEX record')
            record = bytes.fromhex(line[1:])
            if len(record) != record[0] + 5 or sum(record) & 0xff:
                raise ValueError('Invalid HEX length/checksum')
            length, address, rtype = record[0], int.from_bytes(record[1:3], 'big'), record[3]
            data = record[4:4 + length]
            if rtype == 0:
                for i, value in enumerate(data):
                    memory[base + address + i] = value
            elif rtype == 1:
                eof = True
            elif rtype == 2:
                base = int.from_bytes(data, 'big') << 4
            elif rtype == 4:
                base = int.from_bytes(data, 'big') << 16
            elif rtype not in (3, 5):
                raise ValueError('Unsupported HEX record type')
        if not eof or memory != {addr + i: value for addr, data in flash_segments for i, value in enumerate(data)}:
            raise ValueError('HEX does not match ELF flash load segments')
    return dict(profile=profile, crsf='crsf_receiver_start' in symbols,
                flash=flash_end - FLASH_BASE, static_ram=ram_end - RAM_BASE,
                stack=stack, reset=reset)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf', type=Path, nargs='?', default=Path(__file__).resolve().parents[1] / 'bsp/stm32h743/build/fypilot.elf')
    parser.add_argument('--profile', choices=('auto', 'hello', 'imu'), default='auto')
    parser.add_argument('--require-crsf', action='store_true', help='Require CRSF parser, worker and USART6 interrupt')
    args = parser.parse_args()
    try:
        result = check(args.elf, args.profile, args.require_crsf)
    except (OSError, ValueError, struct.error, IndexError) as exc:
        parser.exit(1, f'Firmware verification FAILED: {exc}\n')
    print('Firmware verification PASS: ARM hard-float, flash vectors, target symbols, ELF/BIN/HEX agree')
    print(f'Application profile: {result["profile"]}')
    print(f'CRSF receive path linked: {result["crsf"]}')
    print(f'Flash image {result["flash"]} bytes; static RAM incl. MSP {result["static_ram"]} bytes; '
          f'MSP=0x{result["stack"]:08x}, Reset=0x{result["reset"]:08x}')
