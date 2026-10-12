# Astra 右手闭环 rollout（第一阶段）

入口：`agent/scripts/astra_grasp_rollout.py`。默认目标是桌上的胡萝卜。当前仅实现并开放 pre-grasp：没有 close/lift 动作，没有第二阶段开关。机器人必须已经处于调试模式；入口不调用运动模式切换。未修改 `scripts/record_real.py` 的 X/Y/B/A 逻辑。

## 已有硬件链路

- 只订阅 `FourCameraSubscriber("192.168.123.164", 5555)`。固定索引为 head_left、head_right、wrist_left、wrist_right；索引 2 不传给模型、不保存、不显示。
- Head Stereo 持续负责主要接近和精对准。右腕过曝时不强制切换；头部清晰可判断指间间隙时允许连续 3 次新图确认 pre-grasp。右腕仅在清晰时作为可选精对准视角；目标丢失或不确定仍停止。
- 模型每次只给出一个基座系平移增量和小旋转向量。最大平移 10 mm / 3 mm，最大旋转 0.025 / 0.01 rad（头部 / 腕部）。超限拒绝，不裁剪后继续。
- 与 Pico 遥操作相同，持久保存上一帧指令 `q`，使用其 FK + 当前视觉增量 → 右臂 Placo → 必要时同一目标 DLS 回退 → clamp + 残差、关节、工作空间、自碰撞检查 → 一次 14 维 `ctrl_dual_arm`。实测反馈单独保存，不覆盖指令 `q`；动作检查实际 FK 增量是否跟随，并限制绝对关节跟踪误差，不用静态负载偏差抵消下一步指令。
- 左臂 7 维来自接管时保存的指令；头部指令也保持。腰、底盘不下发。Dex1 使用既有 `set_gripper_ratios(left, 0)`，新增可选 `publish_left=False`，左夹爪不发布命令，避免构造期间打开左手。
- 每个视觉动作的关节变化不超过 0.04 rad，底层原有 0.04 rad / 20 ms 限速不变。等待实测 FK/关节到位后才取接收时间晚于动作结束的新图。
- URDF 提供手指滑动轴、末端帧及机器人自碰撞几何；没有相机标定或目标绝对 3D 位姿计算。

## 只读检查

```bash
conda activate r1_teleop
python agent/scripts/astra_grasp_rollout.py --probe --net enp5s0
```

此命令不创建机械臂/夹爪 publisher，不调用 API，不移动机器人。会保存 head_left、head_right、wrist_right 供检查。

## 接管状态

严格保留“当前指令”需要从此前控制器的 `rt/lowcmd` 读取快照；仅有 lowstate 无法还原之前的指令。现有底层未定义腰部 lowstate 索引，因此必须提供已知固定腰角，不能默认为零。角度只是机器人 FK 状态，不是相机标定。

在原控制器仍保持时读取指令（把 `<腰角>` 替换为实际 rad 值）：

```bash
python agent/scripts/astra_grasp_rollout.py --probe --net enp5s0 \
  --waist-rad <腰角> --save-handover logs/astra_handover.json
```

随后停止旧控制器；新入口检测到仍有 arm/hand 指令发布者会拒绝接管。若没有旧指令，只有明确同意使用静止实测角度作为新的保持指令时才使用 `--hold-measured --waist-rad <腰角>`。该选项不是原指令恢复，不能混称。

## 模型服务与运行

使用 Responses API `/responses`，图像输入 + 严格 JSON Schema。`--model` 默认 `gpt-6-astra`，必须是服务实际支持的准确 ID，不会自动换模型。官方文档访问在此环境失败，未验证该 ID 的 API 可用性。模型别名在本编码会话可用，不等于用户的 API 服务可用。

密钥只从环境读取；默认 `UNITREE_OPENAI_API_KEY`（用户已确认这是 Astra 密钥）。可用 `--api-key-env` 指定其他变量。API 地址和准确模型 ID 仍须与密钥所属服务匹配，变量名本身不能确认服务端点。请求不自动重试、不保存服务端会话。相机图像将发送到配置的视觉服务。

API 请求默认使用本机 Clash `http://127.0.0.1:7897`；`--api-proxy` 可更改，传空字符串可关闭显式代理。只影响 API，不改变 DDS/ZMQ，也不把代理监听端口开放到局域网。用户已指定公司代理 API `https://ys3.chunxueai.com/v1`、模型 `gpt-6-astra`，后续不请求官方 API。

先运行只观察一次提案（不带 `--motion-mode`，不会模拟动作后继续假闭环）：

```bash
python agent/scripts/astra_grasp_rollout.py --net enp5s0 \
  --handover logs/astra_handover.json \
  --model gpt-6-astra --base-url https://ys3.chunxueai.com/v1
```

真机第一阶段：

```bash
python agent/scripts/astra_grasp_rollout.py --motion-mode --enable-dex1 --net enp5s0 \
  --handover logs/astra_handover.json \
  --model gpt-6-astra --base-url https://ys3.chunxueai.com/v1
```

默认工作空间为 URDF pelvis/base 坐标下的 EE 包围盒（单位 m）：X [-0.2, 0.8]，Y [-0.85, 0.05]，Z [-0.45, 0.7]。可用 `--workspace XMIN XMAX YMIN YMAX ZMIN ZMAX` 收紧。它不是桌面模型，也不保证环境无碰撞；环境/桌面间隙依赖视觉判断，URDF 检查只覆盖机器人自身非相邻刚体。系统无法从这些未标定图像提供确定性的外部碰撞保证。

## 停止、记录和恢复

IK 失败、残差大、关节越限/接近限位、自碰撞、工作空间越界、模型不确定、目标或手不可见、图像/反馈过期、模型超时、推理期间画面明显变化均进入锁存保持。不会自动换目标、重新规划、盲目退让或自动回 home。

达到 pre-grasp 或发生错误后进程继续保持 DDS 指令；输入 `home` 或 Ctrl+C 才通过已有 `reset()` / `advance_home_blend()` 回右臂 home，默认 100 帧。左臂、头、腰和左手保持。home 会先检查自碰撞与 EE 工作空间；失败则继续保持，不能瞬移复位。第二次 Ctrl+C 也不能绕过检查。强制杀进程不属于受控恢复。

日志位于 `logs/astra_时间/`，包含使用过的图像、观察、原始决策、已执行动作、故障及恢复事件。没有 wrist_left 图像，没有 API 密钥。最多 60 个循环，超过即保持。默认决策最长 15 秒，超时提案丢弃。

验证：`python -m unittest discover -s tests -v`。单元测试不发布真机指令，覆盖严格动作验证、视角切换、3 帧稳定、禁用左腕、真实 FK/IK、右臂隔离、100 帧 home、夹爪启动、API 结构及失败不重试。

第二阶段须在第一阶段真机可靠性验证后另行实现：闭合 → 新图 → 单步小幅抬起 → 新腕图确认目标跟随；失败张开并保持。当前代码不包含这些动作。


### 30 Hz action interpolation

The Direct rollout executes one validated IK target through right-arm-only joint interpolation at a nominal 30 Hz (33.33 ms). Each visual action takes at least 12 ticks; interpolated joint increments are at most 0.004 rad per tick. All samples are checked for workspace, joint limits and self-collision before the first command. Missed deadlines do not produce catch-up bursts. The existing DDS writer remains at 50 Hz with its original 0.04 rad / 20 ms limit and holds during model inference. This is joint interpolation, not a claim of a straight Cartesian TCP path. No extrapolated actions are generated while waiting for inference. A fresh observation follows each completed action. Timing is best-effort Python scheduling, not hard real time.

Process takeover is disabled after a command-publication gap was observed. The updated Direct controller can resume a latched hold in the same process using a new `resume_rollout.json` in its view directory with the exact `pid` and `requested_at_ns` newer than the hold. This requires an explicit operator continuation request; it is not an automatic retry. Changes on disk do not update a previously running controller.

### Prepared handover and grasp verification

`--prepared-handover-pid` supports a known stationary Direct holder only. The replacement starts with the exact held arm/head command and must successfully publish before terminating the previous process; both briefly emit identical holding commands. Do not use this for a moving controller. Stdout reporting is disabled in the new low-level writer to avoid blocking control on diagnostic output. This mechanism still requires live continuity validation.

`--grasp` enables phase two only after three stationary wrist pregrasp confirmations and a separate visual closure gate. Right Dex1 closes, feedback must move, and new images must confirm capture before lifting. Each lift is 3 mm followed by new images; at most 15 mm is attempted. Success requires both visual following and table separation. Failed verification opens the right hand and latches hold. The Qt preview displays action timing, EEF target pose, commanded/measured deltas, measured control frequency and gripper ratio feedback. The displayed pose is right_dex1_base_link, not a calibrated fingertip TCP.

### Adaptive approach size and per-view freshness

Camera choice no longer forces a 3 mm step: `near_grasp=false` allows a visually clear approach of at most 20 mm with either head or wrist feedback. `near_grasp=true` caps translation at 3 mm in either view. Wrist rotation remains capped at 0.010 rad. Each target still requires new images after execution; no target sequence or extrapolation is generated. The model must mark near-grasp when a 20 mm move could reach contact. Integer bounding-box stability is not a frozen-camera test. The independent freshness check compares each of head_left/head_right/wrist_right pixel arrays across packets and stops after 2 seconds of identical pixels. It never examines wrist_left.

### Current camera selection and raised presentation

The current Direct entry uses ONLY `head_right` and `wrist_right` for inference, detection, freshness checks and the Qt preview. It continues subscribing to the existing four-slot PC2 stream; it does not launch cameras or use either left view. Earlier stereo descriptions above describe the previous setup, not the current Direct policy.

`--grasp --high-lift-m 0.20` requests an additional measured 20 cm lift only AFTER small-lift visual grasp verification. Each increment is at most 20 mm with a fresh clearance decision before movement and fresh following/separation evidence afterward. Workspace, joint limits, collision and feedback checks can stop before the requested height. After verified capture, any elevated-lift failure holds the arm and keeps the right hand closed rather than dropping the object. Initial failed capture still reopens and holds. This behavior has unit coverage; a high lift is not considered hardware-validated until an actual grasp and raise are logged.

### Operator wrist closure criterion

The phase-one policy and phase-two closure verifier share one criterion: the cake's intended grasp area is between the two black fingers and approximately level with both tips in wrist_right. Both `cake_between_fingers` and `tips_at_grasp_height` must be true before pregrasp/closure. Symmetric gaps and perfect centering are not required; intended contact with the cake is allowed. This is a stationary closure criterion, not evidence that an arbitrary arm sweep is clear. Existing freshness, human/table interference, aperture and subsequent visual grasp/lift checks remain active.
