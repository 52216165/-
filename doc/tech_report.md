# 技术文档 — 任务二：桌面小云台自瞄

## 一、系统架构

```
Galaxy相机 → 滑动平均背景差法 → 最大轮廓检测 → P控制 → 串口发云台
               ↑                    ↓
            背景模型(α=0.02)     指数平滑(0.3)
```

## 二、相机标定

### 方法
- 使用 11×8 棋盘格（10×7 内角点），17mm 方块
- OpenCV `findChessboardCorners` + `calibrateCamera`

### 结果
| 参数 | 值 |
|---|---|
| fx | 1721.9 |
| fy | 1720.3 |
| cx | 638.6 |
| cy | 526.5 |
| 重投影误差 | 0.14 px |

## 三、串口通讯

### 协议
- **波特率**: 115200 bps，8N1
- **接收帧**: 43 字节，0x5A 0xA5 帧头
  ```
  [head(2)] [mode(1)] [q[4]×4(16)] [yaw(4)] [yaw_vel(4)] [pitch(4)] [pitch_vel(4)] [bullet_speed(4)] [bullet_count(2)] [tail(2)]
  ```
- **发送帧**: 29 字节，绝对角度（度）

### 关键发现
1. 固件实际不发送 0x7F 0xFE 帧尾，去掉帧尾校验
2. 云台角度单位是**度**，不是弧度（初始误判导致 sanity check 失败）
3. `ov::InferRequest` 不可拷贝，需要 `ThreadSafeQueue::push(T&&)` 移动语义

## 四、目标检测

### YOLO 方案（不适用）
- 用 yolo11n 训练 110 张自采照片
- 置信度最高仅 0.07，远低于可用的 0.3 阈值
- **教训**: 小数据集（<100 张）+ 粗糙标注不适合 YOLO

### 传统方法：滑动平均背景差
- 对每帧灰度图做高斯模糊（15×15）
- 与背景模型比较：`absdiff(gray, bg) → threshold → dilate → findContours`
- 背景更新：`bg = (1-α)×bg + α×gray`，α=0.02
- 优势：慢速更新避免云台自身运动污染背景模型

## 五、控制策略

### P 控制器
| 参数 | 值 | 说明 |
|---|---|---|
| KP_YAW | 0.08 | 偏航增益 |
| KP_PITCH | 0.05 | 俯仰增益 |
| DEADBAND | 10px | 死区，避免抖动 |
| MAX_CMD | 30° | 指令限幅 |
| SMOOTH | 0.3 | 指数平滑防抖 |

### 丢失处理
- 检测丢失后维持最后位置 30 帧（约 2 秒）
- 超过则放弃跟踪，进入 SEARCH 状态
- 重新检测后平滑过渡到新位置

## 六、配置参数

关键配置在 `configs/desktop_gimbal.yaml`：
```yaml
camera_matrix: [1721.9, 0, 638.6, 0, 1720.3, 526.5, 0, 0, 1]
exposure_ms: 10
gain: 16
com_port: "/dev/gimbal"  # 自动回退到 /dev/ttyUSB0
```

## 七、踩坑记录

1. **ROS Jazzy 不兼容**: ament_cmake 在 Ubuntu 24.04 下崩溃，通过 cmake disable 所有 ROS 包解决
2. **Galaxy 相机 SDK**: 图像格式是 BAYER_RG8 需转换，帧率限制 15fps 降低 CPU 负载
3. **云台 IDLE 模式**: 固件默认发送 mode=0（IDLE），覆盖为 AUTO_AIM 后才接受控制指令
4. **磁盘满**: 系统盘 91G 满 → 挂载 1.6T NVMe 分区，符号链接搬迁
