#!/usr/bin/env python3
"""
OTA package bundler (manual trigger, pack-only).

Packs EXISTING assets only — does NOT rebuild web assets:
  - .pio/build/esp32-c3-devkitc-02/firmware.bin
  - data/ (uses the prebuilt data/web/*.gz as-is; excludes cards.json and AGENTS.md)
into dist/update-package.zip.

Use this when only the firmware changed and data/web/*.gz is already up to date.
To rebuild web assets from data_src/web as well, run scripts/release.py instead.

Usage:
    python scripts/pack_ota.py

Prerequisite: firmware.bin must already be built (run `pio run` first).
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile, ZipInfo


PROJECT_ROOT = Path(__file__).resolve().parent.parent

FIRMWARE_PATH = PROJECT_ROOT / ".pio" / "build" / "esp32-c3-devkitc-02" / "firmware.bin"
DATA_DIR = PROJECT_ROOT / "data"
OUTPUT_PATH = PROJECT_ROOT / "dist" / "update-package.zip"

EXCLUDED_FILES = {"cards.json", "AGENTS.md"}


def format_size(size: int) -> str:
    """Human-readable byte size."""
    if size < 1024:
        return f"{size} B"
    if size < 1024 * 1024:
        return f"{size / 1024:.1f} KB"
    return f"{size / (1024 * 1024):.2f} MB"


def collect_data_files(data_dir: Path) -> list[Path]:
    """All files under data_dir, excluding protected device-local files."""
    return [
        p for p in sorted(data_dir.rglob("*"))
        if p.is_file() and p.relative_to(data_dir).as_posix() not in EXCLUDED_FILES
    ]


def write_file(zip_file: ZipFile, source_path: Path, archive_name: str) -> None:
    """Write a file into the zip with deterministic metadata."""
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


def main() -> int:
    print("📦 OTA 升级包打包 (仅打包已有文件，不重建 web)\n")

    if not FIRMWARE_PATH.is_file():
        print(f"❌ 未找到固件: {FIRMWARE_PATH}")
        print("   请先执行 `pio run` 构建固件后再运行本脚本。")
        return 1

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    data_files = collect_data_files(DATA_DIR)

    with ZipFile(OUTPUT_PATH, "w", compression=ZIP_DEFLATED) as zip_file:
        write_file(zip_file, FIRMWARE_PATH, "firmware.bin")
        for source_path in data_files:
            rel = source_path.relative_to(DATA_DIR).as_posix()
            write_file(zip_file, source_path, f"data/{rel}")

    zip_size = OUTPUT_PATH.stat().st_size
    print("─── 汇总 ───")
    print(f"  输出:        {OUTPUT_PATH}")
    print(f"  固件 SHA256: {compute_sha256(FIRMWARE_PATH)}")
    print(f"  数据文件数:  {len(data_files)}")
    print(f"  压缩包大小:  {format_size(zip_size)}")
    print(f"  排除:        {', '.join(sorted(EXCLUDED_FILES))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
