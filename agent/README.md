# Astra 控制模块

`agent/` 包含 R1-A7 的策略、控制适配器、rollout 入口、状态记录和测试。策略只提出单个局部动作；机器人状态读取、确定性安全门控和硬件控制留在本地执行层。

## 当前控制路径

当前 Direct EEF 主入口为 `scripts/astra_direct_rollout.py`：

1. 订阅 PC2 的四槽图像流，并读取 DDS 状态；策略输入、图像新鲜度和预览只使用 `head_right`、`wrist_right`。
2. 读取当前机械臂/头部指令和夹爪状态，构造持续指令快照。创建控制适配器时不创建硬件 publisher。
3. 对每个新观测形成包含图像、目标候选、指令 FK、实测 FK、关节状态和已执行历史的上下文。
4. Astra 返回一个右末端目标及证据字段。策略层拒绝未知/额外字段、关节或轨迹输出、过期/变化场景和不确定提案。
5. 本地控制器将单步末端目标转换为右臂 IK；检查工作空间、关节限制、URDF 自碰撞，并按小步插值发布。
6. 等待动作结束并用实测状态核验，然后写入日志；下一步必须使用动作结束后的新图像。

左臂、头、腰和底盘不由此策略移动。异常会锁存保持；Direct 控制器的继续运行需要同一进程收到明确的操作员恢复请求，不是自动重试。

## 接口与其他路径

| 路径 | 用途 |
|---|---|
| `astra_direct_policy.py` | Direct EEF schema、目标校验、head/wrist 阶段和 pre-grasp 稳定确认 |
| `astra_robot.py` | 右臂控制器适配、FK/IK、碰撞检查和执行反馈 |
| `astra_grasp_stage.py` | 可选闭合、试抬、抓取跟随与抬升验证 |
| `r1a7_sdk/` | 有 capture 绑定和任务阶段门闩的 SDK 接口 |
| `scripts/astra_sdk_rollout.py` | SDK 的独立策略循环，不等同于 Direct EEF schema |
| `scripts/astra_grasp_rollout.py` | 早期阶段一入口及只读 DDS/相机检查工具；能力范围见专页 |

底层复用仓库的 `teleop/` 与 `robot/`。自主策略不需要改写 Pico/XR 遥操采集入口。

## 常用检查

从仓库根目录运行：

```bash
conda activate r1_teleop
python agent/scripts/astra_grasp_rollout.py --probe --net enp5s0
python -m unittest discover -s agent/tests -v
```

Direct 的运行前提、指令接管、停止与记录见 [Astra 闭环 rollout](docs/astra_grasp_rollout.md)。SDK 的数据类型、动作门闩及 episode 记录见 [R1-A7 SDK](docs/r1a7_sdk.md)。
