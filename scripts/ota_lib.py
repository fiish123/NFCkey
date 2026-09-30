#!/usr/bin/env python3
"""
OTA / 网页打包的共用实现（被 pack_ota.py 与本目录其它打包脚本复用）。

包含三步：
  1. build_webui()           在 webui/ 下执行 `pnpm run build`
  2. sync_webui_artifacts()  把 webui/dist 的产物同步到 data/web（仅 .gz + favicon.ico）
     verify_web_payload()    校验 index.html 引用的 /assets/* 资源确实存在
  3. pack_ota()              把 firmware.bin + data/ 打成 dist/update-package.zip
                             （排除 cards.json、AGENTS.md）

单独运行本文件没有意义，它只是库。
"""

from __future__ import annotations

import gzip
import hashlib
import re
import shutil
import subprocess
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile, ZipInfo


# ========== 路径 ==========
PROJECT_ROOT = Path(__file__).resolve().parent.parent

WEBUI_DIR = PROJECT_ROOT / "webui"
WEBUI_DIST_DIR = WEBUI_DIR / "dist"
WEB_DST_DIR = PROJECT_ROOT / "data" / "web"

FIRMWARE_PATH = PROJECT_ROOT / ".pio" / "build" / "esp32-c3-devkitc-02" / "firmware.bin"
DATA_DIR = PROJECT_ROOT / "data"
OUTPUT_PATH = PROJECT_ROOT / "dist" / "update-package.zip"

EXCLUDED_FILES = {"cards.json", "AGENTS.md"}

# index.html 里 /assets/xxx 形式的资源引用
ASSET_REF_RE = re.compile(r"/assets/([A-Za-z0-9._-]+)")


def format_size(size: int) -> str:
    """人类可读的字节数。"""
    if size < 1024:
        return f"{size} B"
    if size < 1024 * 1024:
        return f"{size / 1024:.1f} KB"
    return f"{size / (1024 * 1024):.2f} MB"


def compute_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


# ------------------------------------------------------------------
# 第 1 步：构建 webui
# ------------------------------------------------------------------

def build_webui() -> bool:
    """在 webui/ 下执行 `pnpm run build`。"""
    if not WEBUI_DIR.exists():
        print(f"  ❌ webui 目录不存在: {WEBUI_DIR}")
        return False

    print(f"  webui 目录: {WEBUI_DIR}")
    print("  构建中...\n")

    try:
        result = subprocess.run(
            ["pnpm", "run", "build"],
            cwd=WEBUI_DIR,
            capture_output=True,
            text=True,
            check=True,
        )
        print(result.stdout)

        if not WEBUI_DIST_DIR.exists():
            print(f"  ❌ 构建产物目录不存在: {WEBUI_DIST_DIR}")
            return False
        return True
    except subprocess.CalledProcessError as error:
        print("  ❌ webui 构建失败!")
        print(error.stderr)
        return False
    except FileNotFoundError:
        print("  ❌ 未找到 pnpm 命令，请先安装 pnpm")
        return False


# ------------------------------------------------------------------
# 第 2 步：同步 webui 产物到 data/web
# ------------------------------------------------------------------

def sync_webui_artifacts() -> bool:
    """把 webui/dist 的 .gz 产物与 favicon.ico 复制到 data/web。"""
    if not WEBUI_DIST_DIR.exists():
        print(f"  ❌ webui 构建产物不存在: {WEBUI_DIST_DIR}")
        return False

    print(f"  源目录:   {WEBUI_DIST_DIR}")
    print(f"  目标目录: {WEB_DST_DIR}\n")

    # 清理旧产物（旧的手写多页资源与上一次构建的哈希文件）
    if WEB_DST_DIR.exists():
        print("  🗑️  清理旧文件...")
        for pattern in ["css", "js", "*.html", "favicon.ico"]:
            for old_file in WEB_DST_DIR.glob(pattern):
                if old_file.is_file():
                    old_file.unlink()
                    print(f"     删除: {old_file.name}")
        assets_dir = WEB_DST_DIR / "assets"
        if assets_dir.exists():
            shutil.rmtree(assets_dir)
            print("     删除: assets/")
        print()

    WEB_DST_DIR.mkdir(parents=True, exist_ok=True)
    (WEB_DST_DIR / "assets").mkdir(exist_ok=True)

    copied_files = 0

    index_gz = WEBUI_DIST_DIR / "index.html.gz"
    if index_gz.exists():
        shutil.copy2(index_gz, WEB_DST_DIR / "index.html.gz")
        print(f"  📄 index.html.gz  ({format_size(index_gz.stat().st_size)})")
        copied_files += 1

    assets_src = WEBUI_DIST_DIR / "assets"
    if assets_src.exists():
        for gz_file in sorted(assets_src.glob("*.gz")):
            shutil.copy2(gz_file, WEB_DST_DIR / "assets" / gz_file.name)
            print(f"  📄 assets/{gz_file.name}  ({format_size(gz_file.stat().st_size)})")
            copied_files += 1

    # favicon 不会被 gzip 压缩，单独复制（index.html 引用了 /favicon.ico）
    favicon = WEBUI_DIST_DIR / "favicon.ico"
    if favicon.exists():
        shutil.copy2(favicon, WEB_DST_DIR / "favicon.ico")
        print(f"  📄 favicon.ico  ({format_size(favicon.stat().st_size)})")
        copied_files += 1

    print(f"\n  ✅ 复制了 {copied_files} 个文件")
    return True


def verify_web_payload() -> bool:
    """校验 data/web 自洽：index.html 引用的 /assets/* 必须都在。

    这一步能挡住“index.html 引用了已不存在的旧哈希文件”这类问题——
    设备端的表现是页面白屏（JS 拿不到）。
    """
    index_gz = WEB_DST_DIR / "index.html.gz"
    if not index_gz.is_file():
        print(f"  ❌ 缺少 {index_gz}")
        return False

    try:
        html = gzip.decompress(index_gz.read_bytes()).decode("utf-8", "replace")
    except OSError as error:
        print(f"  ❌ 无法解压 {index_gz}: {error}")
        return False

    refs = sorted(set(ASSET_REF_RE.findall(html)))
    if not refs:
        print("  ⚠️  index.html 中没有 /assets/ 引用，请确认构建产物是否正常")
        return False

    missing = [
        name for name in refs
        if not (WEB_DST_DIR / "assets" / f"{name}.gz").is_file()
        and not (WEB_DST_DIR / "assets" / name).is_file()
    ]
    if missing:
        print("  ❌ index.html 引用了 data/web 中不存在的资源:")
        for name in missing:
            print(f"     - {name}")
        print("     设备端会因拿不到 JS 而白屏，请重新执行同步步骤。")
        return False

    print(f"  ✅ 资源校验通过: index.html 引用的 {len(refs)} 个资源都在")
    return True


# ------------------------------------------------------------------
# 第 3 步：打包 OTA 升级包
# ------------------------------------------------------------------

def collect_data_files(data_dir: Path = DATA_DIR) -> list[Path]:
    """data_dir 下的所有文件，排除设备本地数据文件。"""
    return [
        p for p in sorted(data_dir.rglob("*"))
        if p.is_file() and p.relative_to(data_dir).as_posix() not in EXCLUDED_FILES
    ]


def write_file(zip_file: ZipFile, source_path: Path, archive_name: str) -> None:
    """以固定元数据写入 zip，保证可复现。"""
    zip_info = ZipInfo(archive_name)
    zip_info.compress_type = ZIP_DEFLATED
    zip_info.date_time = (1980, 1, 1, 0, 0, 0)
    zip_info.create_system = 3
    zip_info.external_attr = 0o100644 << 16
    zip_file.writestr(zip_info, source_path.read_bytes())


def pack_ota() -> tuple[int, int] | None:
    """把 firmware.bin + data/ 打包成 dist/update-package.zip。

    返回 (数据文件数, 压缩包字节数)，失败返回 None。
    """
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


def print_summary(file_count: int, zip_size: int) -> None:
    print("─── 汇总 ───")
    print(f"  输出:        {OUTPUT_PATH}")
    print(f"  固件 SHA256: {compute_sha256(FIRMWARE_PATH)}")
    print(f"  数据文件数:  {file_count}")
    print(f"  压缩包大小:  {format_size(zip_size)}")
    print(f"  排除:        {', '.join(sorted(EXCLUDED_FILES))}")


# ------------------------------------------------------------------
# 命令行入口：给 build_webui.sh 复用，保证同步/校验只有一份实现
# ------------------------------------------------------------------

def _cli() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="webui 构建与同步工具")
    parser.add_argument(
        "command",
        choices=["sync"],
        help="sync: 构建 webui 并同步到 data/web（含资源自洽校验）",
    )
    args = parser.parse_args()

    if args.command == "sync":
        if not build_webui():
            print("\n❌ WebUI 构建失败!")
            return 1
        print("\n✅ WebUI 构建完成\n")
        if not sync_webui_artifacts():
            print("\n❌ WebUI 资源同步失败!")
            return 1
        print()
        if not verify_web_payload():
            return 1
        print("\n✅ data/web 已就绪")
        return 0

    return 1


if __name__ == "__main__":
    raise SystemExit(_cli())
