# Astra Direct EEF 闭环 Rollout

本页描述 `agent/scripts/astra_direct_rollout.py` 的实际数据与控制链路。它与早期 `astra_grasp_rollout.py` 的 pre-grasp-only 策略，以及 `astra_sdk_rollout.py` 的 SDK action schema 是不同入口；运行前先确认所用入口。

## 1. 控制所有权与摇操接管

机器人须已由操作员置于调试模式。rollout 不切换 FSM。执行前先用只读 probe 检查相机和 DDS；probe 不创建控制 publisher，不移动机器人：

```bash
python agent/scripts/astra_grasp_rollout.py --probe --net enp5s0
```

Direct rollout 启动时会读取当前 `rt/lowcmd`（若当前没有指令消息，则以关节实测值建立静止保持），并读取头部、腰部及两侧夹爪状态，作为 `RightOnlyController` 的初始指令快照。控制器构造、URDF/IK 和策略安全检查阶段不会创建真实机械臂 writer。左臂、头部、腰和底盘维持快照状态，策略只控制右臂；右 Dex1 的闭合状态也会检查，已闭合时拒绝从开手接近流程重启。

真机 writer 只会在运动模式显式开启、预检查通过后建立。默认要求没有其他命令 publisher。`--prepared-handover-pid` 仅适用于已知、静止且持续发布相同保持命令的 Direct holder：新 writer 先成功发布完全相同的保持指令，再退出旧 holder，避免先停旧进程造成 `lowcmd` 空窗。不可用该方式接管正在运动的控制器。旧式通用 `--takeover-pid` 已禁用。

接管是控制所有权交接，不是把操作者实测姿态直接当作新的运动目标。启动和每步执行前会比较实测关节与保存命令；差异超限、控制器不新鲜或活动 publisher 状态不符合预期时停止。硬件运行前，操作员仍需确保摇操侧已按部署流程静止并处于可交接状态。

## 2. 观测如何形成

图像来自已有 PC2 `FourCameraSubscriber`，槽位为 `head_left`、`head_right`、`wrist_left`、`wrist_right`。当前 Direct 策略只使用 `head_right` 和 `wrist_right`；其余视角不作为策略输入。rollout 还读取 DDS 关节指令和实测反馈，并使用机器人模型计算右末端的 commanded FK 与 measured FK。

每轮使用动作完成后的新帧，观测包括：任务文本、主视角、frame id、当前/实测右末端位姿、右臂实测关节与指令、Dex1 几何、近期已执行动作、工作空间、图像静止检测结果及颜色轮廓候选。轮廓仅是像素候选，不是语义识别、深度或像素到基座坐标的标定。`wrist_left` 不使用。

head_right 用于找目标和较大范围接近；只有连续三次新观测都确认 wrist 视角可用且目标可见，才切换到 wrist-primary 精对准。pre-grasp 还要求 wrist 视角下抓取区域处于两指之间、与指尖高度大致对齐，并连续三次静止确认。

## 3. 决策与确定性校验

Astra 每轮只返回一个右末端目标及状态/证据，不输出关节角、轨迹或物体绝对 3D 位姿。控制坐标为 pelvis/base，末端帧为 `right_dex1_base_link`，它不是指尖 TCP。Direct 策略第一阶段要求夹爪保持打开。

本地策略校验字段、有限数值、置信度、目标/手可见性、路径证据和视角阶段。`move` 单步平移上限为 20 mm；near-grasp 时为 3 mm；头部主视角旋转上限 0.025 rad，腕部主视角 0.010 rad。超大跳变直接拒绝，轻微超限会受限幅；静止状态不得携带非零移动目标。推理期间若机器人位姿变化、相机过期、图像明显变化或决策超时，则提案作废。

随后右臂控制器根据持续保存的指令配置解算 IK。它检查工作空间、关节限位、IK 有效性/残差以及 URDF 自碰撞；离散动作沿右臂关节空间插值，插值采样先通过安全检查，再以约 30 Hz 执行。DDS writer 维持底层约 50 Hz 更新和既有单次关节限速。插值不等于末端笛卡尔直线轨迹；URDF 自碰撞不覆盖桌面或场景障碍物，外部间隙仍依赖视觉证据。

## 4. 下发、反馈与闭环

动作流程为：

```text
新图 + DDS 状态
  → 形成 observation 并记录
  → Astra 单步提案
  → 校验提案、时效及场景稳定性
  → IK / 工作空间 / 限位 / 自碰撞检查
  → 右臂插值下发，其他关节保持
  → 等待反馈稳定并核验 commanded / measured 结果
  → 记录动作前后证据
  → 只接受动作结束后的新图作为下一轮输入
```

模型推理期间不生成外推动作。任何 Halt（例如目标丢失、碰撞不确定、IK 失败、跟随误差、反馈/图像过期、推理超时）都会冻结当前安全指令并记录原因，不盲目退让、重试或自动回 home。Direct 进程保持 writer 存活；恢复必须是操作员针对同一 PID 的明确新请求。不要直接杀进程作为常规恢复方式。

## 5. 抓取阶段与任务边界

默认行为到三次稳定 pre-grasp 确认即结束，夹爪保持打开。传入 `--grasp` 才启用额外阶段：右 Dex1 闭合并核验反馈；闭合后先做小幅纯 Z 试抬；用后续新图确认目标随手移动且离开支撑面；只有验证通过才继续抬升/搬运。闭合成功本身不等于抓取成功。验证失败会按抓取阶段策略处理夹爪并锁存保持。

`--high-lift-m` 请求的是小抬升证据通过后的额外抬升，不是保证达到的高度。每段重新检查路径和工作空间，并在动作后取新图；失败保持右手抓紧，避免丢落物体。此控制器的文档和日志可记录抬升过程，但不能由 dry-run 或单元测试推断硬件 pick-and-place 已成功。

当前 Direct rollout 的目标阶段和成功条件以运行代码及逐次日志为准。完整放置需要有受支撑释放以及释放后新观测验证；若本次运行未到达这些步骤，不应报告完整 pick-and-place 成功。

## 6. 启动、预览与日志

在仓库根目录、确认机器人已处于调试模式后：

```bash
# 无动作：观察并校验单个提案
python agent/scripts/astra_direct_rollout.py --net enp5s0 --view-dir agent/logs/agp_r1a7_view

# 真机 pre-grasp；运动和夹爪启用必须显式给出
python agent/scripts/astra_direct_rollout.py --motion-mode --enable-dex1 --net enp5s0 \
  --view-dir agent/logs/agp_r1a7_view

# 显式允许抓取验证阶段
python agent/scripts/astra_direct_rollout.py --motion-mode --enable-dex1 --grasp --net enp5s0 \
  --view-dir agent/logs/agp_r1a7_view

# 只读预览
DISPLAY=:1 python agent/scripts/right_arm_test_view.py agent/logs/agp_r1a7_view
```

运动期间启用的预览需保持运行且定期更新；预览失活、界面请求停止或 DDS writer 失效会触发保持。每次运行在 `logs/astra_direct_<时间>/` 保存图像、观测、原始提案、安全目标、执行反馈、阶段事件和停止/恢复记录。不要把日志中的“已提案”当作“已执行”；以 `action_settled` 等执行结果记录区分。

旧入口 `astra_grasp_rollout.py` 的能力为 pre-grasp-only，不含当前 Direct `--grasp` 的抓取后验证实现。SDK 路径也有独立状态和接口，参见 [R1-A7 SDK](r1a7_sdk.md)。

## 7. 验证与限制

```bash
python -m unittest discover -s agent/tests -v
```

测试不发布真机动作。视觉为 RGB 单目视角，没有场景深度、手眼标定或确定的外部碰撞证明；Python 调度是尽力而为，不是硬实时控制。真机能力应以实际连续 rollout 日志和操作员核验结果评估。
