# GPT-6 Astra 控制 R1-A7 自主抓取

用 **GPT-6 Astra** 视觉闭环控制 Unitree R1-A7 **右臂 + Dex1**，完成抓取任务。  
策略在 `agent/`；底层 IK / DDS / 相机复用本仓库遥操栈（见文末）。

## Demo

![GPT-6 Astra demo](agent/docs/demo.gif)


## 控制历程

仅用 `head_right` / `wrist_right`。

闭环：取图与 FK → GPT-6 出单步动作 → SDK 门闩校验 → IK/DDS 执行 → 新图再规划。

阶段：`approach → capture → close → trial_lift → raise → transfer → lower → release → verify`

| 动作 | 条件 |
|------|------|
| `close` | 蛋糕在指间且指尖同高、路径清晰 |
| 闭合后首步 | 纯 `+Z` 试抬；新图确认跟随后再搬运 |
| `open` | 盘内已支撑（或空爪失败重试） |
| `done` | 释放后新图确认蛋糕在盘内 |
| 失败 | 保持当前指令，不自动松爪 |

## 项目结构

```txt
r1a7_teleop/
├── agent/                 # GPT-6 Astra 策略、SDK、入口脚本
│   ├── astra_*.py         # 策略 / 右臂控制器 / 抓取阶段
│   ├── r1a7_sdk/          # 观测–动作接口与门闩
│   ├── scripts/           # astra_sdk_rollout 等入口
│   └── docs/              # 说明与 demo.mp4
├── robot/                 # 配置、URDF、DDS、Dex1（被 agent 调用）
├── teleop/                # IK / 真机控制器 / 图像客户端（被 agent 调用）
├── scripts/               # 原遥操入口（本任务不直接跑）
└── assets/r1_a7/          # A7.urdf
```

## 快速开始

```bash
bash setup.sh
conda activate r1_teleop
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
```

机器人端需先起图像与夹爪服务（PC2，如 `./start_teleop.sh`）。控制机：

```bash
# 推荐：SDK 闭环（先 dry-run）
python agent/scripts/astra_sdk_rollout.py --session <session> \
  --view-dir agent/logs/agp_r1a7_view --dry-run

# 可视化
DISPLAY=:1 python agent/scripts/right_arm_test_view.py agent/logs/agp_r1a7_view
```

也可配合 `agent-as-policy` 的 `agp/server_r1a7.py`（硬件）+ `agp/run_r1a7.py`（策略）；`R1A7_TELEOP_ROOT` 指向本仓库根。

API：`https://ys3.chunxueai.com/v1`，模型 `gpt-6-astra`，密钥环境变量 `UNITREE_OPENAI_API_KEY`。

更多细节见 `agent/README.md`、`agent/docs/`。

## 调用的遥操相关模块

本仓库不单独展开 XR 遥操文档。GPT-6 路径实际依赖：

| 模块 | 用途 |
|------|------|
| `teleop/real_controller.py` | `RightOnlyController` 基类 |
| `teleop/placo_solver.py` | 右臂 Placo IK |
| `teleop/ik_solver.py` | IK 回退 / 测试 |
| `teleop/image_client.py` | 四路相机 ZMQ 订阅（用 head_right、wrist_right） |
| `robot/config.py` / `arm_model.py` / `arm_control.py` | 配置、FK、真机臂控 |
| `robot/r1_a7_real.py` | `rt/lowcmd` / `rt/lowstate` DDS |
| `robot/eef/dex1.py` | Dex1 夹爪下发 |

原遥操入口仍在 `scripts/teleop_real.py`、`scripts/teleop_sim.py`，自主抓取任务不需要直接运行它们。
