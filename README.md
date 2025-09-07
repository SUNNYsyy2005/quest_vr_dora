# Quest VR Robot Control System (Dora Clean Version)

这是一个使用Meta Quest VR控制器来控制Piper机械臂的干净版本系统。

## 功能特点

- ✅ VR控制器实时跟踪
- ✅ 6自由度机械臂控制
- ✅ 夹爪控制
- ✅ 位置保持功能（松开B按键后保持当前位置）
- ✅ 流畅的控制响应

## 系统要求

- Meta Quest 2/3 通过USB连接
- Piper机械臂通过CAN总线连接
- Python环境（已配置好vt虚拟环境）

## 快速开始

### 1. 连接硬件
- 通过USB连接Quest（确保开启USB调试）
- 连接Piper机械臂到CAN总线

### 2. 运行系统

**一键启动（推荐）：**
```bash
cd /home/dora/dora-pipers/quest_vr_dora_clean
./start_all.sh
```

这个脚本会自动：
- 激活CAN接口 (can0 @ 1Mbps)
- 激活conda环境 (vt)
- 设置库路径
- 启动Dora coordinator
- 检查Quest连接
- 运行控制系统

**手动启动：**
```bash
# 1. 激活CAN接口
bash can_activate.sh can0 1000000 3-8.4.3:1.0

# 2. 激活环境
conda activate vt
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib"

# 3. 启动Dora
dora up
dora run dataflow.yml
```

### 3. 停止系统

```bash
./stop_all.sh
```

## 控制说明

- **A按钮**: 重置机械臂到初始位置
- **B按钮**: 按住启用控制（松开后保持位置）
- **右扳机**: 控制夹爪开合
- **右手控制器移动**: 控制机械臂末端位置

## 文件结构

```
quest_vr_dora_clean/
├── dataflow.yml                 # Dora数据流配置
├── start_all.sh                 # 一键启动脚本（推荐）
├── stop_all.sh                  # 停止脚本
├── can_activate.sh              # CAN接口激活脚本
├── test_imports.py              # 依赖测试脚本
├── requirements.txt             # Python依赖
├── nodes/                       # 节点实现
│   ├── vr_reader_node.py       # VR数据读取节点
│   ├── ik_solver_exact.py      # IK求解节点
│   ├── robot_controller_node.py # 机器人控制节点
│   ├── oculus_reader.py        # Quest通信库（原版，使用ppadb）
│   ├── buttons_parser.py       # 按钮解析器
│   └── FPS_counter.py          # FPS计数器
└── urdf/                        # 机器人模型文件
    └── piper_description.urdf

```

## 故障排除

### Quest未连接
- 检查USB连接
- 确保Quest开启了USB调试
- 运行 `adb devices` 确认设备连接

### 机械臂不响应
- 检查CAN总线连接
- 确认CAN接口已启用：`sudo ip link set can0 up`
- 查看日志文件：`out/*/log_robot_controller.txt`

### 控制卡顿
- 检查CPU使用率
- 确保没有其他程序占用资源
- 查看IK求解器日志确认没有求解失败

## 技术细节

- 控制频率：50Hz (20ms)
- 通信协议：USB ADB (Quest) + CAN Bus (Robot)
- IK求解器：Pinocchio + CasADi
- 坐标系：右手坐标系

### 关于oculus_reader.py

这是原版的Quest通信库：
- 使用`ppadb` (Pure Python ADB) 库进行Quest通信
- 自动安装Quest VR应用APK
- 支持USB和WiFi连接模式
- 完整的错误处理和设备检测

## 更新历史

- 修复了夹爪控制类型转换问题
- 修复了B按键松开后位置保持问题
- 优化了控制响应速度，减少卡顿
- 改进了IK求解的实时性