import os
import shutil

# 源文件所在的目录（相对于当前目录）
source_dir = "build"

# 源文件名称（相对于source_dir）
source_files = [
    "bootloader/bootloader.bin",
    "xiaozhi.bin",
    "partition_table/partition-table.bin",
    "ota_data_initial.bin",
    "srmodels/srmodels.bin"
]

# 目标文件夹路径（新建文件夹名为"bin"）
target_dir = "bin"

# 如果目标文件夹不存在，则创建它
if not os.path.exists(target_dir):
    os.makedirs(target_dir)

# 复制文件到目标文件夹，如果文件已存在则覆盖
for file_name in source_files:
    # 构建完整的源文件路径
    source_file_path = os.path.join(source_dir, file_name)
    
    # 确保源文件存在
    if os.path.isfile(source_file_path):
        # 构建目标文件路径
        target_file_path = os.path.join(target_dir, os.path.basename(file_name))
        
        # 复制文件
        shutil.copy2(source_file_path, target_file_path)
        print(f"Copied {source_file_path} to {target_file_path}")
    else:
        print(f"Source file not found: {source_file_path}")

print("All files have been processed.")