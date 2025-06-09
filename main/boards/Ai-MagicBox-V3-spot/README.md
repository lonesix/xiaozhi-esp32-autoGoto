# ESP-Spot S3

## 简介

Ai-MagicBox-V2-spot 是基于 ESP-Spot-S3 开发的智能语音交互盒子，内置麦克风、扬声器、IMU 惯性传感器，可使用电池供电。Ai-MagicBox-V2-spot 不带屏幕，带有一个 RGB 指示灯和两个按钮。




## 配置、编译命令

**配置编译目标为 ESP32S3**

```bash
idf.py set-target esp32s3
```

**打开 menuconfig 并配置**

```bash
idf.py menuconfig
```

分别配置如下选项：

- `Xiaozhi Assistant` → `Board Type` → 选择 `Ai-MagicBox-V2-spot`
- `Partition Table` → `Custom partition CSV file` → 输入 `partitions.csv`
- `Serial flasher config` → `Flash size` → 选择 `16 MB`

按 `S` 保存，按 `Q` 退出。

**编译**

```bash
idf.py build
```

**烧录**

```bash
idf.py flash
```


