# Mini ROV 地面站 (B题)

## 项目简介

基于 PySide6 开发的水下机器人地面站控制软件，用于 Mini ROV 的遥控与状态监控。  
功能包括：
- USB 摄像头实时视频显示
- 键盘模拟手柄控制（支持配置文件自定义按键映射）
- 串口通信协议（下行推力/扭矩指令，上行传感器数据解析）
- 过流保护与累计过流时间统计
- 三档速度模式（慢速/中速/快速）
- 机械臂角度控制（0°/45°）

## 通信协议

依据《Mini ROV 水下机器人技术手册》实现。

### 下行数据帧（上位机 → 下位机）

| 字段       | 类型   | 说明                                 |
|-----------|--------|--------------------------------------|
| 帧头       | 0xFA, 0xAF | 固定                                 |
| 帧类型     | 0x49   | ASCII 'I'                           |
| Y 推力     | float  | 小端序，单位 N                       |
| X 推力     | float  | 小端序，单位 N（预留）               |
| Z 推力     | float  | 小端序，单位 N                       |
| Yaw 扭矩   | float  | 小端序，单位 N·m                     |
| 机械臂角度 | uint8  | 0x00 ~ 0xFF，实际使用 0 和 45        |
| 异或校验   | uint8  | 对帧类型 + 数据域所有字节异或        |
| 帧尾       | 0xFB, 0xBF | 固定                                 |

**打包示例 (Python):**
```python
data = struct.pack('<ffffB', y_force, x_force, z_force, yaw_torque, arm_angle)
checksum = 0x49
for b in data:
    checksum ^= b
packet = b'\xFA\xAF\x49' + data + bytes([checksum]) + b'\xFB\xBF'
