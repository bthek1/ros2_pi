#!/usr/bin/env python3
"""Merge colcon's per-package compile databases into one.

colcon writes a `compile_commands.json` per package under `build/<pkg>/`, but
editors want a single file at a predictable path. This concatenates them into
`build/compile_commands.json`, which is what `.vscode/c_cpp_properties.json`
points at.

Run by `just build`; there is no reason to run it by hand.
"""

import json
import sys
from pathlib import Path


def main():
    build = Path(sys.argv[1] if len(sys.argv) > 1 else "build")
    entries = []
    for db in sorted(build.glob("*/compile_commands.json")):
        try:
            entries.extend(json.loads(db.read_text()))
        except (OSError, json.JSONDecodeError) as exc:
            # A half-written database during a failed build is not worth
            # failing the build over — say so and carry on.
            print(f"skipping {db}: {exc}", file=sys.stderr)

    if not entries:
        return 0

    out = build / "compile_commands.json"
    out.write_text(json.dumps(entries, indent=1))
    print(f"{out}: {len(entries)} entries from {len(list(build.glob('*/compile_commands.json')))} packages")
    return 0


if __name__ == "__main__":
    sys.exit(main())
