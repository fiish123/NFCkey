#!/usr/bin/env python3
"""
Release packaging pipeline (manual trigger).

Single pipeline that:
  1. Builds the web filesystem: mirrors data_src/web/ -> data/web/, gzipping
     .html/.css/.js/.bundle assets and pruning stale files.
  2. Packs the OTA update package: bundles .pio/.../firmware.bin + data/ into
     dist/update-package.zip (excluding cards.json and AGENTS.md).

Usage:
    python scripts/release.py

Prerequisite: firmware.bin must already be built (run `pio run` first).
If the firmware is missing, step 1 still completes (web assets are refreshed)
and step 2 aborts with a non-zero exit code.
"""

from __future__ import annotations

import gzip
import hashlib
import os
import shutil
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile, ZipInfo


# ========== Paths ==========
PROJECT_ROOT = Path(__file__).resolve().parent.parent

WEB_SRC_DIR = PROJECT_ROOT / "data_src" / "web"
WEB_DST_DIR = PROJECT_ROOT / "data" / "web"

FIRMWARE_PATH = PROJECT_ROOT / ".pio" / "build" / "esp32-c3-devkitc-02" / "firmware.bin"
DATA_DIR = PROJECT_ROOT / "data"
OUTPUT_PATH = PROJECT_ROOT / "dist" / "update-package.zip"

# ========== Web build config ==========
COMPRESS_EXTENSIONS = {".html", ".css", ".js", ".bundle"}
GZIP_LEVEL = 9

# ========== OTA pack config ==========
EXCLUDED_FILES = {"cards.json", "AGENTS.md"}


def format_size(size: int) -> str:
    """Human-readable byte size."""
    if size < 1024:
        return f"{size} B"
    if size < 1024 * 1024:
        return f"{size / 1024:.1f} KB"
    return f"{size / (1024 * 1024):.2f} MB"


# ------------------------------------------------------------------
# Step 1: build web filesystem
# ------------------------------------------------------------------

def compress_file(src_path: Path, gz_path: Path) -> tuple[int, int, float]:
    """Gzip a single file. Returns (src_size, gz_size, ratio%)."""
    src_size = src_path.stat().st_size
    with src_path.open("rb") as f_in, gzip.open(gz_path, "wb", compresslevel=GZIP_LEVEL) as f_out:
        shutil.copyfileobj(f_in, f_out)
    gz_size = gz_path.stat().st_size
    ratio = (1 - gz_size / src_size) * 100 if src_size > 0 else 0
    return src_size, gz_size, ratio


def build_web_fs() -> bool:
    """Mirror data_src/web into data/web with gzip compression."""
    if not WEB_SRC_DIR.exists():
        print(f"  ❌ 源目录不存在: {WEB_SRC_DIR}")
        return False

    WEB_DST_DIR.mkdir(parents=True, exist_ok=True)

    src_files = {
        Path(root).relative_to(WEB_SRC_DIR) / file
        for root, _, files in os.walk(WEB_SRC_DIR)
        for file in files
    }
    dst_files = {
        Path(root).relative_to(WEB_DST_DIR) / file
        for root, _, files in os.walk(WEB_DST_DIR)
        for file in files
    }

    print(f"  源目录:   {WEB_SRC_DIR}")
    print(f"  目标目录: {WEB_DST_DIR}\n")

    # Copy / compress source files.
    for rel_path in sorted(src_files):
        src_path = WEB_SRC_DIR / rel_path
        dst_path = WEB_DST_DIR / rel_path
        dst_path.parent.mkdir(parents=True, exist_ok=True)

        if src_path.suffix.lower() in COMPRESS_EXTENSIONS:
            gz_path = dst_path.with_name(f"{dst_path.name}.gz")
            src_size, gz_size, ratio = compress_file(src_path, gz_path)
            print(f"  📄 {rel_path}  ({format_size(src_size)} -> {format_size(gz_size)}, -{ratio:.1f}%)")
        else:
            shutil.copy2(src_path, dst_path)
            print(f"  📄 {rel_path}")

    # Prune stale destination files (including orphaned .gz).
    for rel_path in sorted(dst_files):
        if rel_path.suffix == ".gz":
            original = rel_path.with_suffix("")
            if original not in src_files:
                (WEB_DST_DIR / rel_path).unlink()
                print(f"  🗑️  已删除: {rel_path}")
            continue
        if rel_path not in src_files:
            (WEB_DST_DIR / rel_path).unlink()
            print(f"  🗑️  已删除: {rel_path}")

    return True


# ------------------------------------------------------------------
# Step 2: pack OTA update package
# ------------------------------------------------------------------

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


def pack_ota() -> tuple[int, int] | None:
    """Bundle firmware.bin + data/ into dist/update-package.zip."""
    if not FIRMWARE_PATH.is_file():
        print(f"  ❌ 未找到固件: {FIRMWARE_PATH}")
        print("     请先执行 `pio run` 构建固件后再运行本脚本。")
        return None

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    data_files = collect_data_files(DATA_DIR)

    with ZipFile(OUTPUT_PATH, "w", compression=ZIP_DEFLATED) as zip_file:
        write_file(zip_file, FIRMWARE_PATH, "firmware.bin")
        for source_path in data_files:
            rel = source_path.relative_to(DATA_DIR).as_posix()
            write_file(zip_file, source_path, f"data/{rel}")

    return len(data_files), OUTPUT_PATH.stat().st_size


# ------------------------------------------------------------------
# Pipeline
# ------------------------------------------------------------------

def main() -> int:
    print("🚀 Release 打包流程\n")

    print("📦 [1/2] 构建 Web 文件系统")
    if not build_web_fs():
        print("\n❌ Web 文件系统构建失败!")
        return 1
    print("\n✅ Web 文件系统构建完成\n")

    print("📦 [2/2] 打包 OTA 升级包")
    result = pack_ota()
    if result is None:
        print("\n❌ OTA 升级包打包失败!")
        return 1
    file_count, zip_size = result
    print("\n✅ OTA 升级包打包完成\n")

    print("─── 汇总 ───")
    print(f"  输出:        {OUTPUT_PATH}")
    print(f"  固件 SHA256: {compute_sha256(FIRMWARE_PATH)}")
    print(f"  数据文件数:  {file_count}")
    print(f"  压缩包大小:  {format_size(zip_size)}")
    print(f"  排除:        {', '.join(sorted(EXCLUDED_FILES))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
