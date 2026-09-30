#!/bin/bash
# 构建并准备 ESP32 部署的 WebUI

set -e

echo "🔨 构建 WebUI..."
pnpm run build

echo ""
echo "📊 构建统计:"
echo "============================================"
echo "压缩后文件列表:"
find dist -name "*.gz" -type f -exec ls -lh {} \; | awk '{print $5, $9}'

echo ""
echo "总大小统计:"
TOTAL_SIZE=$(find dist -name "*.gz" -type f -exec wc -c {} + | tail -1 | awk '{print $1}')
TOTAL_KB=$(echo "scale=2; $TOTAL_SIZE / 1024" | bc)
echo "  压缩文件总大小: ${TOTAL_KB} KB"
echo "  ESP32 SPIFFS 可用: 1440 KB"
echo "  占用率: $(echo "scale=1; $TOTAL_KB * 100 / 1440" | bc)%"

echo ""
echo "✅ 构建完成！"
echo ""
echo "📦 部署说明:"
echo "  1. 将 dist/ 目录中的所有 .gz 文件复制到 data/web/"
echo "  2. 或者修改固件代码以支持新的目录结构"
echo "  3. 使用 PlatformIO 上传文件系统: pio run -t uploadfs"
