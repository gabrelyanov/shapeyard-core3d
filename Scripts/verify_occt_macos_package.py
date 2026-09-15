#!/usr/bin/env python3
"""Verify the retained, qualified desktop graphics package before shared builds."""
import hashlib
import os
from pathlib import Path
import re
import sys

MANIFEST = "OCCT_MACOS_ARM64_BUILD_MANIFEST.txt"
QUALIFIED_SHA256 = "393320663fbfdd8d4cf1304355a2fc62b55de3e8c24a4d6eb307bd11974267f2"


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def verify(root):
    if not root.is_absolute() or root.resolve(strict=True) != root:
        raise ValueError("package must be an absolute canonical directory")
    manifest = root / MANIFEST
    if manifest.is_symlink() or digest(manifest) != QUALIFIED_SHA256:
        raise ValueError("package does not match the qualified graphics manifest")
    expected = {}
    for line in manifest.read_text().splitlines():
        match = re.fullmatch(r"([0-9a-f]{64})  \./(.+)", line)
        if match:
            sha, name = match.groups()
            if name in expected or Path(name).is_absolute() or ".." in Path(name).parts:
                raise ValueError("invalid inventory path")
            expected[name] = sha
    actual = set()
    links = {}
    for parent, directories, files in os.walk(root, followlinks=False):
        for name in directories + files:
            path = Path(parent) / name
            relative = path.relative_to(root).as_posix()
            if path.is_symlink():
                links[relative] = os.readlink(path)
            elif path.is_file():
                actual.add(relative)
            elif not path.is_dir():
                raise ValueError("unsupported package entry: " + relative)
    if links != {"bin/ExpToCasExe": "ExpToCasExe-7.8.0"}:
        raise ValueError("unexpected package links")
    if actual != set(expected) | {MANIFEST}:
        raise ValueError("package inventory changed")
    for name, sha in expected.items():
        if digest(root / name) != sha:
            raise ValueError("package file changed: " + name)
    print(f"Verified {len(expected)} package files; manifest={QUALIFIED_SHA256}")


if __name__ == "__main__":
    try:
        if len(sys.argv) != 2:
            raise ValueError("usage: verify_occt_macos_package.py /absolute/install")
        verify(Path(sys.argv[1]))
    except (OSError, ValueError) as error:
        print("error: " + str(error), file=sys.stderr)
        sys.exit(78)
