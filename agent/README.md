# GPT-6 Astra / cake control (`agent/`)

GPT-6 相关代码放在本目录（普通包，不是 git worktree）。主仓 `robot/`、`teleop/`、`scripts/teleop_*.py` 保持原遥操入口不变。

## 布局

```txt
agent/
  astra_policy.py / astra_robot.py / astra_direct_policy.py / astra_grasp_stage.py
  cake_detector.py / grasp_criterion.py / visual_freshness.py / ...
  r1a7_sdk/          # import: agent.r1a7_sdk
  scripts/           # import: agent.scripts.*
  docs/
  tests/
```

## 运行（在 r1a7_teleop 仓库根目录）

```bash
conda activate r1_teleop
# 只读可视化
DISPLAY=:1 python agent/scripts/right_arm_test_view.py agent/logs/agp_r1a7_view

# SDK / cake rollout（先 dry-run）
python agent/scripts/astra_sdk_rollout.py --session /path/to/session \
  --view-dir agent/logs/agp_r1a7_view --dry-run
python agent/scripts/fixed_cake_rollout.py --help
python agent/scripts/resident_cake_control.py --help
```

AgP（`agent-as-policy/agp`）通过 `R1A7_TELEOP_ROOT`（默认本仓库根）导入 `agent.*` 与共享 `teleop.image_client` / `robot.config`。
