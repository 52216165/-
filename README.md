# 任务二：桌面小云台自瞄

> Aurora 27 届视觉组暑训考核 — 方向A 自瞄

## 项目地址
- **GitHub**: https://github.com/52216165/- (task2 分支)
- **任务一（整车考核）**: 同仓库 `main` 分支

## 项目概述
独立编写桌面小云台视觉跟踪系统。

目标：矿石（ore），1 类目标检测 + P 控制跟踪。

## 环境
- Ubuntu 24.04 + OpenCV 4.x + OpenVINO 2024.6
- Galaxy MER-139-210U3C 相机
- 桌面云台（Lracking 固件，USB 转 TTL 串口）

## 构建
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON \
  -DCMAKE_DISABLE_FIND_PACKAGE_rclcpp=ON
make -j$(nproc) desktop_gimbal
```

## 运行
```bash
export LD_LIBRARY_PATH=/opt/intel/openvino_2024.6.0/runtime/lib/intel64:$LD_LIBRARY_PATH
./build/desktop_gimbal              # 带云台控制
./build/desktop_gimbal --no-gimbal  # 仅视觉调试
```

## 关键文件
```
src/desktop_gimbal.cpp         ——SimpleTracker 主程序
io/gimbal/gimbal.cpp/hpp       ——串口通讯（0x5A 0xA5 协议）
io/galaxy/galaxy.cpp/hpp       ——Galaxy 相机驱动
tasks/auto_aim/yolos/yolo26n.cpp/hpp ——YOLO OpenVINO 后处理
configs/desktop_gimbal.yaml    ——配置文件
assets/ore_model.xml/.bin      ——训练好的 YOLO 模型
```

## 检测方案
- **YOLO one-stage**: yolo26n, OpenVINO CPU 推理, mAP50=0.995（Roboflow polygon 标注）
- **传统 fallback**: 滑动平均背景差法 (α=0.02) + 轮廓检测

## 控制策略
| 参数 | 值 |
|---|---|
| KP_YAW | 0.08 |
| KP_PITCH | 0.05 |
| 死区 | 10px |
| 限幅 | ±30° |
| 平滑 | 0.3 指数平滑 |
| 超时 | 30 帧 |

## 相机标定
- 11×8 棋盘格（17mm）
- fx=1721.9, fy=1720.3, cx=638.6, cy=526.5
- 重投影误差 0.14px
