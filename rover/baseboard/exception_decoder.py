#!/usr/bin/env python3
# Standalone replacement for PlatformIO's filter_exception_decoder.py
# Reads text from stdin, looks for hex addresses and runs addr2line to
# produce a human-readable backtrace. Usage:
#   cat crash.log | python filter_exception_decoder_standalone.py -e /path/to/elf
#
# Options:
#   -e, --elf PATH         : path to ELF/firmware file (required)
#   -a, --addr2line PATH   : explicit addr2line path (optional)
#   -s, --strip-prefix STR : strip this prefix from file paths in output
#   -q, --quiet            : suppress stderr informational messages

import argparse
import os
import re
import shlex
import shutil
import subprocess
import sys

ADDR_PATTERN = re.compile(r"((?:0x[0-9a-fA-F]{8}[: ]?)+)")
ADDR_SPLIT = re.compile(r"[ :]")
PREFIX_RE = re.compile(r"^ *")
IS_WINDOWS = sys.platform.startswith("win")


def find_addr2line(provided=None):
    if provided:
        if os.path.isfile(provided) and os.access(provided, os.X_OK):
            return provided
        # try to resolve via PATH if not absolute
        p = shutil.which(provided)
        if p:
            return p
        return None
    # common candidates
    candidates = [
        "addr2line",
        "xtensa-esp32-elf-addr2line",
        "xtensa-esp32s2-elf-addr2line",
        "xtensa-esp32s3-elf-addr2line",
        "riscv32-unknown-elf-addr2line",
        "arm-none-eabi-addr2line",
    ]
    for c in candidates:
        p = shutil.which(c)
        if p:
            return p
    return None


def is_address_ignored(address):
    return address in ("", "0x00000000")


def filter_addresses(addresses_str):
    addresses = ADDR_SPLIT.split(addresses_str)
    size = len(addresses)
    while size > 1 and is_address_ignored(addresses[size - 1]):
        size -= 1
    return addresses[:size]


def strip_project_dir(trace, prefix):
    if not prefix:
        return trace
    return trace.replace(prefix, "")


def build_backtrace_for_addresses(addr2line_path, elf_path, addresses, strip_prefix=None, quiet=False):
    if not addresses:
        return ""
    prefix = ""
    trace_out = ""
    enc = "mbcs" if IS_WINDOWS else "utf-8"

    i = 0
    for addr in addresses:
        addr = addr.strip()
        if is_address_ignored(addr):
            continue
        try:
            # call addr2line for single address to keep output mapping simple
            args = [addr2line_path, "-fipC", "-e", elf_path, addr]
            output = subprocess.check_output(args).decode(enc).strip()
        except subprocess.CalledProcessError as e:
            if not quiet:
                sys.stderr.write(
                    "failed to call %s: %s\n" % (addr2line_path, e)
                )
            continue

        # normalize output (indent inlined function newlines)
        output = output.replace("\n", "\n     ")

        # skip unknown entries
        if output == "?? ??:0":
            continue

        output = strip_project_dir(output, strip_prefix)
        trace_out += "%s  #%-2d %s in %s\n" % (prefix, i, addr, output)
        i += 1

    return trace_out + ("\n" if trace_out else "")


def process_stream(input_text, addr2line_path, elf_path, strip_prefix=None, quiet=False):
    out_lines = []
    buffer = ""
    idx = 0
    while True:
        # iterate line by line, preserving newlines
        next_nl = input_text.find("\n", idx)
        if next_nl == -1:
            # remaining chunk
            tail = input_text[idx:]
            if tail:
                buffer += tail
            break
        line = input_text[idx:next_nl]
        if buffer:
            line = buffer + line
            buffer = ""
        idx = next_nl + 1

        m = ADDR_PATTERN.search(line)
        if m is None:
            out_lines.append(line + "\n")
            continue

        # found addresses; build trace and insert after this line
        addresses = filter_addresses(m.group(1))
        trace = build_backtrace_for_addresses(addr2line_path, elf_path, addresses, strip_prefix, quiet)
        out_lines.append(line + "\n")
        if trace:
            out_lines.append(trace)

    # append any leftover buffer
    if buffer:
        out_lines.append(buffer)

    return "".join(out_lines)


def main():
    parser = argparse.ArgumentParser(
        description="Standalone exception decoder: read from stdin, use addr2line to decode addresses."
    )
    parser.add_argument("-e", "--elf", required=True, help="Path to ELF/firmware file")
    parser.add_argument("-a", "--addr2line", required=False, help="Path to addr2line binary (optional)")
    parser.add_argument("-s", "--strip-prefix", required=False, default="", help="Strip this prefix from file paths")
    parser.add_argument("-q", "--quiet", action="store_true", help="Suppress stderr messages")
    args = parser.parse_args()

    elf_path = os.path.abspath(args.elf)
    if not os.path.isfile(elf_path):
        sys.stderr.write("ELF file not found: %s\n" % elf_path)
        return 2

    addr2line_path = find_addr2line(args.addr2line)
    if not addr2line_path:
        sys.stderr.write("addr2line binary not found. Provide via --addr2line or install addr2line in PATH.\n")
        return 3

    # Read stdin
    try:
        input_text = sys.stdin.read()
    except KeyboardInterrupt:
        return 0

    output = process_stream(input_text, addr2line_path, elf_path, args.strip_prefix, args.quiet)
    sys.stdout.write(output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
