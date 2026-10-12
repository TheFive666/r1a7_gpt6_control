# R1-A7 遥操作与数据采集框架

基于 **Unitree R1-A7 人形机器人** 的 XR 遥操作框架, 支持 PICO VR 半身遥操数据采集。

## 项目结构

```txt
r1a7_teleop/
├── robot/                       # R1 机器人专用代码
│   ├── config.py                # 配置、URDF 路径、关节定义、初始位姿常量
│   ├── arm_model.py             # Pinocchio 模型加载与工具函数
│   ├── arm_control.py           # 手臂控制器（仿真/真机）
│   ├── r1_a7_real.py            # 真机 DDS 桥（rt/lowcmd + lowstate）
│   ├── eef/                     # 末端执行器驱动
│   │   ├── dex1.py              # Dex1 夹爪（set_gripper_ratios + 力限位 latch）
│   │   └── dex1_controller.py   # Dex1 夹爪控制（Dex1Controller）
│   ├── gripper.py               # Dex1 夹爪仿真控制（GripperController）
│   ├── urdf_utils.py            # URDF 预处理（肘关节下限 override）
│   ├── visualization.py         # meshcat 可视化
│   └── xr_interface.py          # XR 设备接口
├── teleop/                      # 通用遥操作框架
│   ├── placo_solver.py          # PlacoIKSolver（Placo QP IK 封装）
│   ├── ik_solver.py             # 阻尼最小二乘 IK 求解器（迭代 + 正则）
│   ├── retargeting.py           # 人体姿态映射 / mock XR 位姿
│   ├── xr_client.py             # XRoboToolkit 客户端封装
│   ├── r1a7_controller_base.py  # 共享 tracking+IK
│   ├── sim_controller.py        # SimTeleopController
│   ├── real_controller.py       # RealTeleopController
│   ├── lerobot_v21_writer.py    # LeRobot Dataset v2.1 四相机写入器
│   └── utils/
│       └── geometry.py          # 头显/手柄坐标系变换与四元数工具
├── scripts/                     # 可执行脚本
│   ├── teleop_sim.py            # 仿真遥操作入口
│   └── teleop_real.py           # 真机遥操作入口
├── assets/
│   └── r1_a7/
│       └── A7.urdf              # R1-A7 URDF 模型
├── agent/                       # GPT-6 Astra 抓取控制（独立包，见 agent/README.md）
├── third_party/                 # Unitree SDK / XRoboToolkit（setup.sh 拉取）
├── setup.sh                     # 环境安装脚本
└── start_env.md                 # 代理说明、仿真测试、常见问题
```


## 快速开始

### 环境准备

```bash
bash setup.sh
conda activate r1_teleop
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
```

### 仿真遥操作

```bash
# mock XR（不需要 PICO）；Meshcat 默认开启
python scripts/teleop_sim.py --mock-xr

# 带调试输出
python scripts/teleop_sim.py --mock-xr --debug

# 关掉可视化
python scripts/teleop_sim.py --mock-xr --no-is-vis

# 连接 PICO，在 VR 中遥操作，可以不加 debug
python scripts/teleop_sim.py --debug
```

### 真机遥操作

```bash
python scripts/teleop_real.py --motion-mode --enable-dex1 --net enp131s0
```

### 数据采集 / 回放

```bash
# 开启机器人端图像服务端和夹爪驱动
ssh unitree@192.168.123.164
conda activate teleop
./start_teleop.sh

# 采集（确保 PC2 image server 已启动）
python scripts/record_real.py \
  --motion-mode \
  --enable-dex1 \
  --image-server-address 192.168.123.164 \
  --net enp131s0

# 采集为 LeRobot Dataset v2.1（其余控制参数相同）
python scripts/record_real.py \
  --record-format lerobot-v2.1 \
  --lerobot-repo-id local/r1_a7_dex1 \
  --task-dir-name test \
  --frequency 30 \
  --motion-mode \
  --enable-dex1 \
  --image-server-address 192.168.123.164 \
  --net enp131s0


# 回放校验
python scripts/replay_mcap.py --episode <path>/episode_0000.mcap

# 只校验格式，不开窗口
python scripts/replay_mcap.py --episode <path>/episode_0000.mcap --no-video
```

检查 PC2 四路图像的发布频率、各相机有效新帧率并显示四宫格预览：

```bash
python scripts/check_image_stream.py --address 192.168.123.164

# 无桌面环境时只统计 10 秒，不打开窗口
python scripts/check_image_stream.py --address 192.168.123.164 --no-show --duration 10
```

### Debug

若 PC2 未安装 `tcpdump`，使用脚本内置的原始 UDP 探测（不依赖 GStreamer 解码）：

```bash
python3 ~/check_official_head_stream.py --probe-only --duration 10
```

若 PC2 没有任何数据，官方流很可能只在头部计算机 PC1（`192.168.123.161`）
本机提供；将测试脚本复制到 PC1：

```bash
scp scripts/check_official_head_stream.py unitree@192.168.123.161:/home/unitree/
ssh unitree@192.168.123.161

# 有桌面显示：左右目并排预览，Ctrl+C 退出
python3 ~/check_official_head_stream.py

# 无桌面显示：只统计 15 秒帧率
python3 ~/check_official_head_stream.py --no-show --duration 15
```

**Pico 手柄按键**

| 按键 | 功能 |
|------|------|
| X | 进入 / 退出遥操作模式（退出时双臂平滑回初始位） |
| Y | 开始 / 停止并保存一条 episode（需先进入遥操作模式） |
| B | 暂停 / 恢复当前 episode 的写盘（同时暂停手臂输出） |
| A | 删除最近保存的一条 episode（仅空闲时可用，只能删一次） |
| 侧握 grip | 激活对应手臂的末端跟踪 |
| 扳机 trigger | 控制 Dex1 夹爪开合 |

> 录制模式下 A 键被占用为「删除」，因此基类的 A 键复位功能会自动关闭。

**数据格式**
`--record-format` 支持 `mcap`（默认，兼容原有数据）和 `lerobot-v2.1`。

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--scale-factor` | 1.0 | IK 缩放因子 |
| `--elbow-lower-override` | -1.30 | 肘关节下限覆盖 |
| `--frequency` | 30.0 | 数据采集帧率（Hz） |
| `--record-format` | mcap | `mcap` 或 `lerobot-v2.1` |
| `--lerobot-repo-id` | local/r1_a7_dex1 | LeRobot 数据集逻辑 repo id（不会自动上传） |
| `--image-server-address` | 192.168.123.164 | PC2 图像服务地址 |


### GPT-6 Astra 控制自主抓取历程

代码在 `agent/`（详见 `agent/README.md`）。任务：黄蛋糕 → 蓝盘；仅用 `head_right` / `wrist_right`，右臂 + Dex1。

闭环：取图与 FK → GPT-6 出单步动作 → SDK 门闩校验 → IK/DDS 执行 → 新图再规划。

阶段：`approach → capture → close → trial_lift → raise → transfer → lower → release → verify`

| 动作 | 条件 |
|------|------|
| `close` | 蛋糕在指间且指尖同高、路径清晰 |
| 闭合后首步 | 纯 `+Z` 试抬；新图确认跟随后再搬运 |
| `open` | 盘内已支撑（或空爪失败重试） |
| `done` | 释放后新图确认蛋糕在盘内 |
| 失败 | 保持当前指令，不自动松爪 |

```bash
python agent/scripts/astra_sdk_rollout.py --session <session> \
  --view-dir agent/logs/agp_r1a7_view [--dry-run]