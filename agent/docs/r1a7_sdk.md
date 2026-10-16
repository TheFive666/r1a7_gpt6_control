# R1-A7 Agent SDK

这个 SDK 是 GPT-6 Astra 与 R1-A7 控制栈之间的唯一上层接口。Rollout
代码不应 import `robot.*`、`teleop.*`，也不应读取 IK、DDS、Dex1 或相机实现。

## 接口边界

模型负责：看 `top` / `wrist` 图像，判断当前证据，规划一个邻近末端增量。

SDK 和 server 负责：capture 绑定、动作门闩、FK/IK、工作空间、关节限位、
自碰撞、插值限速、DDS、新鲜度检查和故障保持。

模型只需要知道两个坐标约定：`dpos_m` 在 `pelvis_link` 中表达，`+X` 前、
`+Y` 左、`+Z` 上；`drot_deg` 是基座系外禀 xyz 旋转。

## 最小 rollout

```python
from agent.r1a7_sdk import Action, Evidence, R1A7PickPlaceSDK

robot = R1A7PickPlaceSDK("agp/sessions/r1_trial")
observation = robot.observe()

# 把 observation.images 中的 top / wrist 图像和 ACTION_SCHEMA 交给 Astra。
# 每轮只解析一个结构化 action；SDK 执行后自动返回新观测。
proposal = {
    "action": "move",
    "stage": "approach",
    "dpos_m": [0.0, -0.01, 0.0],
    "drot_deg": [0.0, 0.0, 0.0],
    "evidence": {
        "confidence": 0.95,
        "uncertain": False,
        "path_clear": True,
        "cake_between_fingers": False,
        "tips_at_grasp_height": False,
        "object_follows_hand": False,
        "object_supported": False,
        "inside_plate": False,
        "placement_verified": False,
        "empty_gripper": False,
    },
    "observation": "蛋糕在夹爪右侧，路径清晰",
    "reason": "向右小步对准",
}
result = robot.step(proposal)
observation = result.observation
```

正式循环必须保持同一个 `R1A7PickPlaceSDK` 实例，因为它保存“已闭合、已试抬、
抓取已验证、已释放”等门闩。不要为每一步重建实例。

## Astra 唯一需要的动作

| action | 含义 |
|---|---|
| `move` | 一个不超过 20 mm / 0.025 rad 的末端增量 |
| `close` | 视觉确认目标位于指间且指尖同高后闭合 |
| `open` | 物体受支撑且位于盘内后释放 |
| `observe` | 不动作，只刷新两幅图像 |
| `done` | 新图确认放置完成 |
| `stop` | 不确定、遮挡、目标丢失或路径不清时保持 |

闭合后的第一步运动必须是 `+Z <= 3 mm` 的纯试抬。下一幅新图必须令
`object_follows_hand=true`，SDK 才允许继续搬运。`step()` 默认在真实动作完成后
自动调用 `observe()`，因此返回值可以直接进入下一轮推理。

## 控制历程与上下文

参考 RPent 的 step state、recipe 和 compact ledger 分层，rollout 不再把原始
bridge event 或整段历史 proposal 塞回模型：

```python
import time
from agent.r1a7_sdk import ControlLedger, EpisodeTrace

ledger = ControlLedger("yellow cake -> blue plate", max_steps=60)
trace = EpisodeTrace(session)

before = observation
started = time.monotonic()
result = robot.step(proposal)
action = Action.from_dict(proposal)
ledger.update(action, result, robot.task_state)
trace.append(step=ledger.step, before=before, action=action, result=result,
             ledger=ledger, elapsed_s=time.monotonic() - started)

# 下一轮只给 Astra：ledger.context()、当前 robot state 和两幅当前图像。
planner_context = ledger.context()
observation = result.observation
```

`sdk_episode/steps.jsonl` 严格对齐每一步的动作前观测、proposal、执行结果和
动作后观测；`sdk_episode/recipe.jsonl` 只保存实际执行过的动作，可用于复盘，
但不能作为绝对坐标轨迹盲目重放。

ledger 始终只保留最近四步摘要，并显式给出 `held_object`、
`first_unmet_gate`、`blocker` 和 `remaining_steps`。模型应执行第一个未满足门闩，
每步检查新图；接近成功时只修复剩余 blocker，不重新开始完整任务。

## Schema 与自检

```bash
python agent/scripts/r1a7_sdk.py --session agp/sessions/r1_trial schema
python agent/scripts/r1a7_sdk.py --session agp/sessions/r1_trial status
python agent/scripts/r1a7_sdk.py --session agp/sessions/r1_trial observe
```

Python 中直接使用 `r1a7_sdk.ACTION_SCHEMA` 作为结构化输出的严格 JSON
schema。总体 rollout 入口和 Direct / SDK 路径区别见仓库根目录 `README.md`；SDK 不创建 publisher，
也不负责切换机器人 FSM。

推荐的完整 Astra 入口是：

```bash
python agent/scripts/astra_sdk_rollout.py --session agp/sessions/r1_trial --dry-run
```

真机去掉 `--dry-run`。该入口不读取 RPent，也不依赖其运行时；这里只移植了其
primitive/state dump/compact ledger 的控制原则。

Rollout 会在第一次机器人观测前启动 `/dev/video0`，以 5 FPS 录制旁观画面到
`<session>/sdk_episode/external_camera.mp4`。该视频明确标记为
`policy_input: false`，不会发送给 Astra，也不参与动作证据判断。配套的
`external_camera.json` 保存设备、帧率和起止时间，FFmpeg 日志写入
`external_camera_ffmpeg.log`。

```bash
# 改帧率或设备
python agent/scripts/astra_sdk_rollout.py --session <session> \
  --external-camera /dev/video0 --external-fps 3

# 仅在相机维护或离线测试时显式禁用
python agent/scripts/astra_sdk_rollout.py --session <session> --no-external-recording
```
