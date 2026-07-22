# Aurora 27 届视觉组暑训考核 — 任务一：整车考核

## 项目概述
基于同济开源自瞄 [sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25/) 适配 Ubuntu 24.04，完成整车串口通讯、相机标定、YOLO 目标检测、云台跟踪与击打。

## 环境配置
- **系统**: Ubuntu 24.04
- **编译器**: g++ 13+
- **CMake**: ≥ 3.20
- **构建**: `mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)`

### 依赖
- OpenCV 4.x
- OpenVINO 2024.6 (用于 YOLO 推理加速)
- Eigen3
- yaml-cpp
- fmt
- Ceres Solver
- Galaxy SDK (大恒相机驱动)

### ROS 2 说明
ROS 2 环境已禁用（在 Ubuntu 24.04 + ROS Jazzy 下 ament_cmake 不兼容），通过 cmake 选项跳过：
```
-DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON
-DCMAKE_DISABLE_FIND_PACKAGE_rclcpp=ON
-DCMAKE_DISABLE_FIND_PACKAGE_std_msgs=ON
-DCMAKE_DISABLE_FIND_PACKAGE_rosidl_typesupport_cpp=ON
-DCMAKE_DISABLE_FIND_PACKAGE_sp_msgs=ON
```

## 已完成功能

### 1. 相机标定
- **相机**: 大恒水星 MER-139-210U3C (Galaxy SDK)
- **标定板**: 11×8 棋盘格 (17mm 格子)
- **标定结果**: 10 张图像，重投影误差 0.14px
- **工具**: `capture_desktop`（采集）/ `calibrate_checkerboard`（计算）
- **结果文件**: `configs/desktop_gimbal.yaml`

### 2. 串口通讯（下位机协议）
- **波特率**: 115200 bps, 8N1
- **接收帧**: 43 字节，包含四元数、yaw/pitch 角度、角速度、弹速等
- **发送帧**: 29 字节，控制云台 yaw/pitch 角度（绝对角度，单位度）
- **协议**: 匹配 Lracking 固件 (0x5A 0xA5 帧头)
- **设备**: `/dev/ttyUSB0`（自动检测）
- **源码**: `io/gimbal/gimbal.cpp`, `io/gimbal/gimbal.hpp`

### 3. YOLO 目标检测
- **模型**: yolo26n (OpenVINO IR 格式)
- **推理后端**: OpenVINO (CPU)
- **输入尺寸**: 640×640
- **类别**: 1 类 (ore/矿石)
- **数据集**: 110 张自采照片标注训练

### 4. 云台跟踪
- **方法**: 滑动平均背景差法检测运动目标 + P 控制
- **平滑**: 指数平滑防抖
- **丢失处理**: 30 帧超时放弃跟踪
- **限位**: ±30° 安全范围

## 关键文件
```
├── src/desktop_gimbal.cpp      # 主程序入口
├── io/
│   ├── gimbal/gimbal.cpp       # 云台串口通讯
│   └── galaxy/galaxy.cpp       # 相机驱动
├── tasks/auto_aim/
│   ├── yolos/yolo26n.cpp       # YOLO 后处理
│   └── multithread/mt_detector.cpp  # 多线程检测器
├── configs/desktop_gimbal.yaml # 配置文件
├── calibration/
│   ├── capture_desktop.cpp     # 标定图像采集
│   └── calibrate_checkerboard.cpp # 标定计算
└── assets/
    ├── ore_model.xml           # YOLO 模型 (已训练)
    └── yolo26n.bin
```

## 提交截止
2026.07.25 23:59
