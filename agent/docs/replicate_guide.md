# GPT-6 Astra Agent-as-Policy · Unitree R1-A7

本文是在 `agent-as-policy`（YAM AgP）思路上，落地到本仓库 R1-A7 的**可执行规范**。  
实现以 `/home/karthus_chen/ycb_ws/agent-as-policy/agp/{server_r1a7,run_r1a7}.py` 为准，底层控制复用本仓库，不重写遥操栈。

上层 rollout 新代码应优先使用 [`agent.r1a7_sdk`](r1a7_sdk.md)。它把本文的
capture、证据门闩和 bridge 协议封装为 `observe()` / `step()`，Astra 无需搜索
或理解底层库。

---

## 0. 一句话目标

```text
GPT-6 Astra = 视觉 + 推理 + 下一步右手意图
R1-A7 现有栈  = FK / IK / 限速 / DDS / Dex1
```

每轮只做一件事：**观察 → 一个邻近动作 → 真机执行 → 再观察**。  
禁止：标定 → 绝对物体 XYZ → 一次开环轨迹。

---

## 1. 控制范围

| 控制 | 不动 |
|------|------|
| Right Arm 7 关节 | Left Arm / Left Dex1 |
| Right Dex1 | Waist / Head / Base |

末端帧：`right_dex1_base_link`（不是指尖 TCP）。  
基座系：`pelvis_link`，`+X` 前、`+Y` 左、`+Z` 上。

---

## 2. 对照原版 AgP：必须照搬 vs 必须改掉

| 原版 YAM AgP | R1-A7 本项目 |
|--------------|--------------|
| 文件桥 `robot_client.py` + 持久 server | **照搬**：`agp/server_r1a7.py` + `agp/run_r1a7.py` |
| Agent 输出 tool action，不是关节 | **照搬**：只允许 `move_delta` / `gripper` / `observe` / `done` / `stop` |
| 一步一小动作，再取新图 | **照搬** |
| 确定性护栏在 server，不在模型 | **照搬** |
| 顶相机 + 腕相机 + 深度 `deproject` | **去掉**：无深度、无标定、无 deproject |
| `move_ee` / `move_joints` / `home` / program | **去掉**：不支持 |
| 校准 overhead `top` | **改成**：`top` = PC2 `head_right` RGB 别名 |
| `wrist` = 腕 D405 RGB-D | **改成**：`wrist` = PC2 `wrist_right` RGB |
| YAM grasp_site | **改成**：`right_dex1_base_link` FK |

不要再按 YAM 手册去起 `hardware-bridge`、跑手眼标定或 base transform。

---

## 3. 视觉输入（以当前 Direct/AgP 为准）

只订阅已有 PC2 流：

```text
FourCameraSubscriber("192.168.123.164", 5555)
索引：0 head_left | 1 head_right | 2 wrist_left | 3 wrist_right
```

**推理 / 新鲜度 / 预览只用：**

```text
head_right  （工具别名 top）
wrist_right （工具别名 wrist）
```

- 不使用 `wrist_left`、不使用 `head_left`。  
- 不在控制机重新开相机，不直接读 RTP/UDP。  
- RGB only：无深度、无像素反投影、无手眼矩阵。

分工（软提示，不是硬状态机强制切换）：

```text
head_right  → 全局找目标、粗接近、抬起/搬运时的相对证据
wrist_right → 近距对准、闭合判据、抓取跟随确认
```

---

## 4. Agent 输出：增量，不是绝对位姿

**正确（与 `run_r1a7.py` 一致）：**

```json
{
  "action": "move_delta",
  "stage": "approach",
  "dpos": [0.0, -0.01, 0.0],
  "drot_deg": [0.0, 0.0, 0.0],
  "confidence": 0.9,
  "uncertain": false,
  "path_clear": true,
  "cake_between_fingers": false,
  "tips_at_grasp_height": false,
  "object_follows_hand": false,
  "object_supported": false,
  "inside_plate": false,
  "placement_verified": false,
  "empty_gripper": false,
  "observation": "...",
  "reason": "..."
}
```

- `dpos`：基座系米，单步 `‖dpos‖ ≤ 20 mm`；近接触时建议 `≤ 3 mm`。  
- `drot_deg`：基座系外禀 xyz 度，转成 rotvec 后 `≤ 0.025 rad`。  
- 非运动动作（`close`/`open`/`observe`/`done`/`stop`）的 delta **必须为零**。  
- Astra **禁止**直接输出 7 关节角或整段轨迹。

**错误（旧稿里的绝对 Target 写法，不要用）：**

```text
right_eef_target = {x,y,z, qw,qx,qy,qz}   ← 不要再作为主接口
```

若内部仍把 delta 变成目标位姿，只允许：

```text
target = FK(commanded_q) ⊕ delta
```

其中 `commanded_q` 是**持久指令配置**，不是把实测反馈覆盖进指令。

---

## 5. 闭环（硬规则）

```text
frames(top, wrist)          # 新鲜、未消费的 capture
    ↓
Astra → 一个 action
    ↓
Server 护栏（delta / 证据 / 阶段）
    ↓
IK（仅右臂）→ 关节插值下发
    ↓
等待到位 + 新图时间戳晚于动作结束
    ↓
下一轮；禁止在等推理时外推动作
```

必须始终区分：

```text
Requested EEF ≠ Measured EEF
Commanded q  ≠ Measured q
```

实测只用于验证与观测，不得抹掉指令记忆。

---

## 6. 底层复用（本仓库，不要重写）

| 能力 | 位置 |
|------|------|
| FK | `robot/arm_model.link_pose` → `right_dex1_base_link` |
| IK | Placo 优先，`solve_upper_arm_ik` 回退；只改 `RIGHT_ARM_JOINTS` |
| 右臂适配 | `agent/astra_robot.RightOnlyController` |
| 下发 | `ctrl_dual_arm([left7_hold, right7_new])` → DDS `rt/lowcmd` |
| 夹爪 | `Dex1.set_gripper_ratios(left_hold, right)`，`0=开 1=关` |
| 图像 | `teleop/image_client.FourCameraSubscriber` |
| 限速 | 低层 `max_joint_delta=0.04 rad / 20 ms` **不得绕过** |

关于「单步关节变化过大」：

- **不要**再用软件 soft limit（如 0.10 rad）拒绝已合法的 IK 目标。  
- 小 EE 步在奇异附近可能需要较大关节变化；平滑交给低层限速。  
- 仍必须 Halt：IK 无解 / 残差过大 / 关节越限 / 工作空间 / 自碰撞 / 视觉不确定。

真机参数与现有一致：`--motion-mode --enable-dex1 --net enp5s0`。  
机器人须已在操作员确认的调试模式；本栈**不**切换 FSM。

---

## 7. 确定性护栏（模型不能绕过）

Server / executor 至少检查：

1. Delta 幅值（20 mm / 0.025 rad；近接触更严）  
2. Workspace（默认 pelvis EE 盒：X[-0.2,0.8] Y[-0.85,0.05] Z[-0.45,0.7]）  
3. 关节限位与贴限位  
4. IK 有效性与残差  
5. URDF 自碰撞（不保证桌面/环境碰撞）  
6. 图像与 DDS 新鲜度；推理期间画面剧变 → Halt  
7. 每个动作绑定**未使用过的** `capture`  
8. 证据布尔：不确定 / 目标丢失 / path 不清 → `stop` 并保持  

故障行为（对齐 AgP）：

```text
Halt → 锁存保持当前臂指令 + 保持当前夹爪状态
```

- **不要**故障就自动回 home（抓着物体时会摔）。  
- Home blend（`reset` + `advance_home_blend`，默认 100 帧）仅在操作员明确请求恢复时使用。  
- 禁止根据「上一帧目标在右边」在丢视后继续盲走。

---

## 8. 任务阶段：证据门控，不是硬编码 FSM

不要写死 `SEARCH → HEAD → WRIST → …` 强制切换。  
由 Astra 报告 `stage` + 证据，**代码做门闩**：

推荐任务（与 `run_r1a7.py` 一致）：**黄蛋糕 → 蓝色盘子**。

| 阶段 | 允许动作 | 关键门闩 |
|------|----------|----------|
| approach / capture | `move_delta`，夹爪开 | 近接触用 ≤3 mm；闭合前须 `cake_between_fingers` 且 `tips_at_grasp_height` |
| close | `close` | 未闭合 + path_clear + 上述两布尔 |
| trial_lift | 闭合后**第一刀**必须是 ≤3 mm 纯 +Z | 不要求抬前就证明 following |
| raise / transfer | `move_delta`，夹爪保持闭 | 试抬后须新图 `object_follows_hand` 才算 grasp_verified |
| lower / release | `open` | 须 `object_supported` 且 `inside_plate`（空抓重试除外） |
| verify / done | `done` | 新图确认仍在盘内且已离指 |

腕部闭合判据（与 Direct 共用）：

> 目标抓取区域在两根黑手指之间，并大致与指尖同高。  
> 不要求左右间隙完全对称；允许与蛋糕预期接触。

成功关闭 ≠ 成功抓取；成功打开 ≠ 成功放置。  
`done` 必须靠**新图证据**，不能靠愿望。

---

## 9. 推荐落地路径（已有代码，按此跑）

在 `agent-as-policy`：

```bash
# 1) 只读 server（无 publisher）
python agp/server_r1a7.py --session agp/sessions/r1_trial --net enp5s0 \
  --view-dir /home/karthus_chen/ycb_ws/r1a7_teleop/agent/logs/agp_r1a7_view

# 2) 取帧自检
python agp/robot_client.py agp/sessions/r1_trial --arm right frames \
  '{"cams":["top","wrist"]}'

# 3) dry-run 策略环
python agp/run_r1a7.py --session agp/sessions/r1_trial --dry-run \
  --view-dir /home/karthus_chen/ycb_ws/r1a7_teleop/agent/logs/agp_r1a7_view
```

真机：新建 session，server 加 `--motion-mode --enable-dex1`；  
禁止为通过「无竞争 publisher」检查而杀掉仍在 hold 的进程（避免 lowcmd 空窗）。

API（本环境）：

```text
base_url = https://ys3.chunxueai.com/v1
model    = gpt-6-astra
key env  = UNITREE_OPENAI_API_KEY
proxy    = http://127.0.0.1:7897
```

本仓库并行入口（同一底层）：

- Direct 闭环：`agent/scripts/astra_direct_rollout.py` / `agent/scripts/astra_grasp_rollout.py`  
- 细节：`docs/astra_grasp_rollout.md`  
- **不要改** `scripts/record_real.py` 的 Pico X/Y/B/A 采集逻辑。

---

## 10. 软件边界

```text
agent-as-policy/agp/
  server_r1a7.py     # 硬件所有权 + 文件桥 + 护栏
  run_r1a7.py        # Astra 观察 / tool 循环
  robot_client.py    # 原 AgP CLI，不改协议
  test_r1a7.py

r1a7_teleop/
  agent/astra_robot.py          # 右臂 FK/IK/下发
  agent/astra_policy.py         # Schema / Halt / Phase 工具
  teleop/image_client.py         # 四相机订阅
  robot/eef/dex1.py              # 夹爪
```

原则：

```text
Reuse R1 infrastructure. Do not rewrite teleop. Do not port YAM calib.
```

---

## 11. 完整数据流

```text
PC2 Image Server
      ↓
FourCameraSubscriber
      ↓  (只用 head_right, wrist_right)
AgP frames tool  →  capture id + RGB + commanded/measured state
      ↓
GPT-6 Astra  (move_delta | close | open | observe | done | stop)
      ↓
server_r1a7 护栏 + 证据门闩
      ↓
RightOnlyController.solve_step → Placo/DLS
      ↓
14D ctrl_dual_arm + Dex1
      ↓
R1-A7 → 新图 + 新状态 → 下一轮
```

---

## 12. 核心原则（精简版）

1. Astra 管意图（邻近 delta / 开合），不管关节。  
2. IK 与护栏是确定性执行接口，模型不能覆盖。  
3. 一观测 → 一动作 → 新观测；禁止开环轨迹与推理期外推。  
4. 不标定、不算绝对物体 XYZ；靠视觉闭环纠偏。  
5. `Requested ≠ Measured`；指令记忆与实测反馈分离。  
6. 失败保持（含夹爪），不自动 home、不盲走。  
7. 抓取/放置用证据门闩，不用愿望当 success。  

---

## 13. 旧稿纠偏清单（写 prompt / 实现时自查）

| 旧稿说法 | 应改为 |
|----------|--------|
| Astra 输出绝对 EEF pose + quaternion | 输出 `dpos` + `drot_deg` 增量 |
| 使用 head_left + head_right 立体 | 当前只用 `head_right` + `wrist_right` |
| 硬编码 SEARCH→WRIST→PREGRASP FSM | Agent `stage` + 代码证据门闩 |
| 失败自动 Home blend | Halt 保持；home 仅人工恢复 |
| 软件拒绝「关节变化 >0.1 rad」 | 取消 soft limit；保留真安全 Halt |
| 新建一整套 agent/control 目录 | 优先扩展已有 `agp/*_r1a7.py` |
| 复用 YAM deproject / calib | 明确不支持 |

---

## 14. 验收

硬件任务未完成，直到 session 日志里真实出现：

```text
close → 试抬跟随确认 → 搬运 → 盘内支撑释放 → 新图 placement_verified
```

单元 / 契约：`python -m unittest`（本仓库 Astra 测试）+ `agp/test_r1a7.py`。  
Dry-run **不得**声称真机抓取成功。

---


---

## 15. 固定轨迹复现：蛋糕放入蓝色盘子

1. UI 界面在 rollout 前开启，执行中持续更新观测、规划、Delta、状态、阶段和 Step。
2. 每次规划只包含一个动作，规划文字、下发 Delta、任务阶段和当前 Step 必须一致。
3. 每次平移 Delta 必须为 2cm，一段 Delta 就是一个 Step。
4. 控制直接调用已有 `PlacoIKSolver.solve()`，不添加工作空间、关节位置、碰撞位置或视觉门控约束。
5. 一次运行必须完成完整 rollout，UI 中的流程和 Step 从开始到结束连续更新。
6. EEF 轨迹：抬高 6cm → 向前 16cm 到蛋糕 → Pitch 朝下 30 度 → 闭合夹爪抓取 → 抬高 10cm → 向左 20cm 到盘子上方 → 向下 6cm → 打开夹爪放入盘子 → 结束。
