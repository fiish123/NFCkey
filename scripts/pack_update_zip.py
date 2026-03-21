#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import sys
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile, ZipInfo


PROJECT_ROOT = Path(__file__).resolve().parent.parent
FIRMWARE_PATH = PROJECT_ROOT / ".pio" / "build" / "esp32-c3-devkitc-02" / "firmware.bin"
DATA_DIR = PROJECT_ROOT / "data"
OUTPUT_PATH = PROJECT_ROOT / "dist" / "update-package.zip"
EXCLUDED_FILES = {
    "cards.json",
    "AGENTS.md",
}


def resolve_firmware_path() -> Path:
    if not FIRMWARE_PATH.exists() or not FIRMWARE_PATH.is_file():
        raise ValueError(f"Firmware file not found: {FIRMWARE_PATH}")
    return FIRMWARE_PATH


def resolve_data_dir() -> Path:
    if not DATA_DIR.exists() or not DATA_DIR.is_dir():
        raise ValueError(f"Data directory not found: {DATA_DIR}")
    return DATA_DIR


def should_include(relative_path: Path) -> bool:
    return relative_path.as_posix() not in EXCLUDED_FILES


def collect_data_files(data_dir: Path) -> list[Path]:
    files: list[Path] = []
    for path in sorted(p for p in data_dir.rglob("*") if p.is_file()):
        if should_include(path.relative_to(data_dir)):
            files.append(path)
    return files


def write_file(zip_file: ZipFile, source_path: Path, archive_name: str) -> None:
    zip_info = ZipInfo(archive_name)
    zip_info.compress_type = ZIP_DEFLATED
    zip_info.date_time = (1980, 1, 1, 0, 0, 0)
    zip_info.create_system = 3
    zip_info.external_attr = 0o100644 << 16
    zip_file.writestr(zip_info, source_path.read_bytes())


def compute_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def create_package(firmware_path: Path, data_dir: Path) -> tuple[int, int]:
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    data_files = collect_data_files(data_dir)

    with ZipFile(OUTPUT_PATH, "w", compression=ZIP_DEFLATED) as zip_file:
        write_file(zip_file, firmware_path, "firmware.bin")
        for source_path in data_files:
            relative_path = source_path.relative_to(data_dir).as_posix()
            write_file(zip_file, source_path, f"data/{relative_path}")

    return len(data_files), OUTPUT_PATH.stat().st_size


def main() -> int:
    try:
        firmware_path = resolve_firmware_path()
        data_dir = resolve_data_dir()
        file_count, zip_size = create_package(firmware_path, data_dir)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Created: {OUTPUT_PATH}")
    print(f"Firmware: {firmware_path} ({compute_sha256(firmware_path)})")
    print(f"Packaged data files: {file_count}")
    print(f"ZIP size: {zip_size} bytes")
    print("Excluded: data/cards.json, data/AGENTS.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
