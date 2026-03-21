#!/usr/bin/env python3
"""
Web文件系统自动构建脚本
功能：将data_src/web/的源文件复制到data/web/，并生成.gz压缩文件
"""

import os
import gzip
import shutil
import inspect
from pathlib import Path
import sys

print("🚀 Web文件系统压缩工具")



# 检查是否在执行文件系统上传
def is_filesystem_upload():
    
    cmd_args = ' '.join(sys.argv).lower()
    if 'uploadfs' in cmd_args or 'buildfs' in cmd_args:
        return True
    
    return False


# 如果不是文件系统上传，直接退出
isskip=False
if not is_filesystem_upload():
    print("⏭️  跳过 Web 文件系统构建")
    isskip=True
    



# ========== 配置 ==========
try:
    current_file = __file__
except NameError:
    current_file = inspect.getfile(inspect.currentframe())

# 项目根目录
PROJECT_ROOT = Path(current_file).parent.parent
SRC_DIR = PROJECT_ROOT / "data_src" / "web"
DST_DIR = PROJECT_ROOT / "data" / "web"

# 需要压缩的文件扩展名
COMPRESS_EXTENSIONS = {'.html', '.css', '.js','.bundle'}

# 压缩级别 (1-9, 9最高但最慢)
GZIP_LEVEL = 9

def format_size(size):
    """格式化文件大小"""
    if size < 1024:
        return f"{size} B"
    elif size < 1024 * 1024:
        return f"{size / 1024:.1f} KB"
    else:
        return f"{size / (1024 * 1024):.2f} MB"

def compress_file(src_path, dst_path):
    """压缩单个文件"""
    src_size = os.path.getsize(src_path)
    
    with open(src_path, 'rb') as f_in:
        with gzip.open(dst_path, 'wb', compresslevel=GZIP_LEVEL) as f_out:
            shutil.copyfileobj(f_in, f_out)
    
    dst_size = os.path.getsize(dst_path)
    ratio = (1 - dst_size / src_size) * 100 if src_size > 0 else 0
    
    return src_size, dst_size, ratio

def copy_file(src_path, dst_path):
    """复制单个文件"""
    shutil.copy2(src_path, dst_path)
    return os.path.getsize(dst_path)

def sync_directories():
    """同步源目录到目标目录"""
    if not SRC_DIR.exists():
        print(f"❌ 源目录不存在: {SRC_DIR}")
        return False
    
    # 确保目标目录存在
    DST_DIR.mkdir(parents=True, exist_ok=True)
    
    
    # 收集源目录所有文件的相对路径
    src_files = set()
    for root, dirs, files in os.walk(SRC_DIR):
        for file in files:
            rel_path = Path(root).relative_to(SRC_DIR) / file
            src_files.add(rel_path)
    
    # 收集目标目录所有文件的相对路径
    dst_files = set()
    for root, dirs, files in os.walk(DST_DIR):
        for file in files:
            rel_path = Path(root).relative_to(DST_DIR) / file
            dst_files.add(rel_path)
    
    # 复制和压缩源文件
    print(f"\n📦 开始处理文件...")
    print(f"   源目录: {SRC_DIR}")
    print(f"   目标目录: {DST_DIR}")
    print()
    
    for rel_path in sorted(src_files):
        src_path = SRC_DIR / rel_path
        dst_path = DST_DIR / rel_path
        
        # 确保目标目录存在
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        
        # 复制文件
        if src_path.suffix.lower() in COMPRESS_EXTENSIONS:
            # 压缩文件
            src_size, gz_size, ratio = compress_file(src_path, f"{dst_path}.gz")
            print(f"   📄 {rel_path}")
            print(f"      原始: {format_size(src_size)} -> 压缩: {format_size(gz_size)} ({ratio:.1f}%)")
        else:
            # 直接复制
            copy_file(src_path, dst_path)
            print(f"   📄 {rel_path}")
    
    # 删除目标目录中不再存在的文件
    for rel_path in dst_files:
        # 跳过.gz文件（它们是自动生成的）
        if rel_path.suffix == '.gz':
            # 检查对应的原始文件是否还存在
            original_path = rel_path.with_suffix('')
            if original_path not in src_files:
                gz_path = DST_DIR / rel_path
                gz_path.unlink()
            continue
        
        # 检查原始文件是否存在
        if rel_path not in src_files:
            dst_path = DST_DIR / rel_path
            if dst_path.is_file():
                dst_path.unlink()
                print(f"   🗑️  已删除: {rel_path}")
    
    return True


if (isskip==False):
    success = sync_directories()
    if success:
        print("\n✅ 构建完成!")
        # PlatformIO 中不要使用 exit(0)，以免中断构建
    else:
        print("\n❌ 构建失败!")
        exit(1)
