#!/bin/bash
# WebUI 构建和部署脚本
# 用于自动化 webui 前端构建和固件编译流程

set -e

echo "=========================================="
echo "  NFCKey WebUI 构建和部署"
echo "=========================================="
echo ""

# 1+2. 构建 WebUI 并同步到 data/web
#      复用 scripts/ota_lib.py，构建/同步/资源校验与 pack_ota.py 走同一份实现
echo "[1/3] 构建 WebUI 并同步到 data/web..."
cd "$(dirname "$0")"
python3 scripts/ota_lib.py sync
echo "   data/web/ 大小: $(du -sh data/web/ | cut -f1)"
echo ""

# 3. 编译固件
echo "[2/3] 编译固件..."
~/.platformio/penv/bin/pio run
echo "✅ 固件编译完成"
echo ""

# 4. 构建 LittleFS 文件系统
echo "[3/3] 构建 LittleFS 文件系统..."
~/.platformio/penv/bin/pio run --target buildfs
echo "✅ 文件系统构建完成"
echo ""

echo "=========================================="
echo "  构建完成！"
echo "=========================================="
echo ""
echo "固件位置: .pio/build/esp32-c3-devkitc-02/firmware.bin"
echo "文件系统: .pio/build/esp32-c3-devkitc-02/littlefs.bin"
echo ""
echo "烧录命令："
echo "  固件: pio run --target upload"
echo "  文件系统: pio run --target uploadfs"
echo ""
