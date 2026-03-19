#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.decode_cbor import CBORDecoder


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Extract CBOR byte payloads into files")
    parser.add_argument("input", type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("decoded"))
    args = parser.parse_args()

    data = args.input.read_bytes()
    values = CBORDecoder(data).decode()[0]
    byte_entries = [entry for entry in values if isinstance(entry, list) and len(entry) >= 5 and isinstance(entry[-1], (bytes, bytearray))]

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    manifest = []
    for idx, entry in enumerate(byte_entries):
        payload = bytes(entry[-1])
        outfile = out_dir / f"chunk_{idx:02d}.bin"
        outfile.write_bytes(payload)
        manifest.append({
            "index": idx,
            "code": entry[0],
            "fields": entry[1:-1],
            "size": len(payload),
            "path": str(outfile.resolve()),
        })

    (out_dir / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"Extracted {len(manifest)} byte payloads into {out_dir}")


if __name__ == "__main__":
    main()
