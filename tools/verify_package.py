#!/usr/bin/env python3
"""Verify this source archive; does not import or execute robot code."""
import hashlib
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[1]
manifest = root / 'MANIFEST.sha256'
failures = []
count = 0
for line in manifest.read_text(encoding='utf-8').splitlines():
    expected, name = line.split('  ', 1)
    path = (root / name).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        failures.append(name + ': missing or unsafe path')
        continue
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != expected:
        failures.append(name + ': SHA-256 mismatch')
    count += 1
if failures:
    print('\n'.join(failures))
    sys.exit(1)
print(f'PASS: {count} files verified; no hardware code executed.')
