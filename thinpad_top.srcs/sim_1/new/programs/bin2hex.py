#!/usr/bin/env python3
"""Convert a binary dump into a Verilog $readmemh-compatible hex file.

The WaitBoot monitor image published by the grader (chunk_00.bin) is a raw
little-endian BaseRAM snapshot. This script rewrites it into one 32-bit word per
line so our test bench can load it directly.
"""

from __future__ import annotations

import argparse
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        required=True,
        type=Path,
        help="Path to the binary blob exported from the grader",
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="Destination path for the Verilog hex file",
    )
    return parser.parse_args()


def convert(bin_path: Path, hex_path: Path) -> None:
    data = bin_path.read_bytes()
    if len(data) % 4:
        raise ValueError(
            f"Binary length {len(data)} is not word-aligned; expected multiples of 4 bytes"
        )

    words = len(data) // 4
    hex_path.parent.mkdir(parents=True, exist_ok=True)

    with hex_path.open("w", encoding="ascii") as outf:
        for idx in range(words):
            word_bytes = data[4 * idx : 4 * idx + 4]
            value = int.from_bytes(word_bytes, byteorder="little")
            outf.write(f"{value:08x}\n")

    print(f"Wrote {words} words to {hex_path}")


def main() -> None:
    args = parse_args()
    convert(args.input, args.output)


if __name__ == "__main__":
    main()
