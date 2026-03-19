#!/usr/bin/env python3
"""Minimal CBOR decoder tailored for Thinpad Cloud submission files."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Iterable, List


def _bytes_to_jsonable(data: bytes) -> dict[str, Any]:
    return {"__bytes__": data.hex()}


def _to_jsonable(obj: Any) -> Any:
    if isinstance(obj, dict):
        return {str(k): _to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_to_jsonable(x) for x in obj]
    if isinstance(obj, bytes):
        return _bytes_to_jsonable(obj)
    return obj


class CBORDecoder:
    def __init__(self, data: bytes):
        self._buf = memoryview(data)
        self._pos = 0

    def _ensure(self, n: int) -> None:
        if self._pos + n > len(self._buf):
            raise EOFError("unexpected end of CBOR data")

    def _read(self, n: int) -> memoryview:
        self._ensure(n)
        start = self._pos
        self._pos += n
        return self._buf[start:self._pos]

    def _read_byte(self) -> int:
        return int(self._read(1)[0])

    def _peek_break(self) -> bool:
        return self._pos < len(self._buf) and self._buf[self._pos] == 0xFF

    def _read_uint(self, add_info: int) -> int:
        if add_info < 24:
            return add_info
        if add_info == 24:
            return self._read_byte()
        if add_info == 25:
            return int.from_bytes(self._read(2), "big")
        if add_info == 26:
            return int.from_bytes(self._read(4), "big")
        if add_info == 27:
            return int.from_bytes(self._read(8), "big")
        raise ValueError(f"indefinite length not allowed here (add={add_info})")

    def _read_length(self, add_info: int) -> int | None:
        if add_info < 24:
            return add_info
        if add_info == 24:
            return self._read_byte()
        if add_info == 25:
            return int.from_bytes(self._read(2), "big")
        if add_info == 26:
            return int.from_bytes(self._read(4), "big")
        if add_info == 27:
            return int.from_bytes(self._read(8), "big")
        if add_info == 31:
            return None
        raise ValueError(f"invalid additional information: {add_info}")

    def _half_to_float(self, half: int) -> float:
        sign = (half >> 15) & 0x1
        exp = (half >> 10) & 0x1F
        frac = half & 0x3FF
        if exp == 0:
            val = frac / (1 << 10) * (2 ** -14)
        elif exp == 0x1F:
            if frac == 0:
                return math.inf if sign == 0 else -math.inf
            return math.nan
        else:
            val = (1 + frac / (1 << 10)) * (2 ** (exp - 15))
        return -val if sign else val

    def decode(self) -> list[Any]:
        values: List[Any] = []
        while self._pos < len(self._buf):
            values.append(self._decode_item())
        return values

    def _decode_item(self) -> Any:
        initial = self._read_byte()
        major = initial >> 5
        add = initial & 0x1F

        if major == 0:  # unsigned int
            return self._read_uint(add)
        if major == 1:  # negative int
            return -1 - self._read_uint(add)
        if major == 2:  # byte string
            length = self._read_length(add)
            if length is None:
                chunks = []
                while not self._peek_break():
                    chunk = self._decode_item()
                    if not isinstance(chunk, (bytes, bytearray, memoryview)):
                        raise ValueError("indefinite byte string expects byte chunks")
                    chunks.append(bytes(chunk))
                self._pos += 1  # consume break
                return b"".join(chunks)
            return bytes(self._read(length))
        if major == 3:  # text string
            length = self._read_length(add)
            if length is None:
                parts: list[str] = []
                while not self._peek_break():
                    fragment = self._decode_item()
                    if not isinstance(fragment, str):
                        raise ValueError("indefinite text expects string chunks")
                    parts.append(fragment)
                self._pos += 1
                return "".join(parts)
            return self._read(length).tobytes().decode("utf-8")
        if major == 4:  # array
            length = self._read_length(add)
            items = []
            if length is None:
                while not self._peek_break():
                    items.append(self._decode_item())
                self._pos += 1
                return items
            for _ in range(length):
                items.append(self._decode_item())
            return items
        if major == 5:  # map
            length = self._read_length(add)
            items = {}
            if length is None:
                while not self._peek_break():
                    key = self._decode_item()
                    value = self._decode_item()
                    items[key] = value
                self._pos += 1
                return items
            for _ in range(length):
                key = self._decode_item()
                value = self._decode_item()
                items[key] = value
            return items
        if major == 6:  # tagged
            tag = self._read_uint(add)
            value = self._decode_item()
            return {"__tag__": tag, "value": value}
        if major == 7:
            if add < 20:
                return {"__simple__": add}
            if add == 20:
                return False
            if add == 21:
                return True
            if add == 22:
                return None
            if add == 23:
                return {"__undefined__": True}
            if add == 24:
                return {"__simple__": self._read_byte()}
            if add == 25:
                half = int.from_bytes(self._read(2), "big")
                return self._half_to_float(half)
            if add == 26:
                import struct

                return struct.unpack("!f", self._read(4))[0]
            if add == 27:
                import struct

                return struct.unpack("!d", self._read(8))[0]
            if add == 31:
                raise ValueError("unexpected break byte outside indefinite context")
        raise ValueError(f"unsupported CBOR major={major} add={add}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Decode Thinpad CBOR logs")
    parser.add_argument("input", type=Path, help="Path to the .cbor file")
    parser.add_argument("--dump-json", dest="dump_json", type=Path, help="Optional path to dump converted JSON")
    parser.add_argument("--max-preview", type=int, default=10, help="Number of top-level items to preview")
    args = parser.parse_args()

    data = args.input.read_bytes()
    decoder = CBORDecoder(data)
    values = decoder.decode()

    print(f"Decoded {len(values)} top-level CBOR items from {args.input}.")
    for idx, item in enumerate(values[: args.max_preview]):
        print(f"[{idx}] {type(item).__name__}: {repr(item)[:120]}")

    if args.dump_json:
        jsonable = _to_jsonable(values)
        args.dump_json.write_text(json.dumps(jsonable, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"JSON dump written to {args.dump_json}")


if __name__ == "__main__":
    main()
