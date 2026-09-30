#!/usr/bin/env python3
"""
OTA 升级包打包（默认会先构建 webui，避免把过期的网页打进包里）。

流程：
  1. 构建 webui：在 webui/ 下执行 `pnpm run build`
  2. 同步产物：webui/dist/*.gz + favicon.ico -> data/web/，并校验
     index.html 引用的 /assets/* 资源齐全
  3. 打包：.pio/build/esp32-c3-devkitc-02/firmware.bin + data/
     -> dist/update-package.zip（排除 cards.json、AGENTS.md）

用法：
    python scripts/pack_ota.py              # 构建网页 + 打包（推荐）
    python scripts/pack_ota.py --skip-web   # 跳过网页构建，直接用 data/web 现有产物
                                            # （只改固件时更快，仍会校验资源自洽）

前置条件：firmware.bin 已构建（先执行 `pio run`）。
"""

from __future__ import annotations

import argparse

import ota_lib


def main() -> int:
    parser = argparse.ArgumentParser(description="打包 OTA 升级包（含 webui 构建）")
    parser.add_argument(
        "--skip-web",
        action="store_true",
        help="跳过 webui 构建与同步，直接使用 data/web 现有产物",
    )
    args = parser.parse_args()

    print("📦 OTA 升级包打包\n")

    if args.skip_web:
        print("⏭  跳过 WebUI 构建（使用 data/web 现有产物）\n")
    else:
        print("📦 [1/3] 构建 WebUI")
        if not ota_lib.build_webui():
            print("\n❌ WebUI 构建失败!")
            return 1
        print("\n✅ WebUI 构建完成\n")

        print("📦 [2/3] 同步 WebUI 资源到 data/web")
        if not ota_lib.sync_webui_artifacts():
            print("\n❌ WebUI 资源同步失败!")
            return 1
        print("\n✅ WebUI 资源同步完成\n")

    # 无论是否重新构建，都校验一次 data/web 与 index.html 引用是否一致
    if not ota_lib.verify_web_payload():
        return 1
    print()

    print("📦 [3/3] 打包 OTA 升级包")
    result = ota_lib.pack_ota()
    if result is None:
        print("\n❌ OTA 升级包打包失败!")
        return 1
    print("\n✅ OTA 升级包打包完成\n")

    ota_lib.print_summary(*result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
