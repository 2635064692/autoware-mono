---
mode: plan
cwd: /opt/work/autoware-mono
task: Planning Simulator 下实现 Bus 20km/h 跟随，稳态 3m 间距（规划/实施/验证）
complexity: medium
planning_method: builtin
created_at: 2026-01-07T16:57:46+08:00
---

# Plan: Bus 跟随 20km/h，稳态 3m 间距

🎯 任务概述
- 目标是在 Planning Simulator 中，通过 RViz 放置 Bus（20km/h）作为前车，自车能够稳定跟随并在稳态维持约 3m 跟车距离。
- 本任务优先采用“最小可行改动”：先通过参数配置实现目标；若稳定性或可复用性不足，再升级为“场景包化/策略参数化”的小范围改动。

📋 执行计划
1. 明确验收口径与基线复现：按任务文档启动 `planning_simulator.launch.xml`，放置 Bus，并订阅 `planning_info`，记录默认配置下的目标距离与误差，作为对照基线。
2. 方案选择（最小可行优先）：
   - ✅ 选择：**方案 B（更可追踪/不污染默认配置）** —— 新增一份场景专用参数文件，并通过 launch 参数链路将 `motion_velocity_planner_obstacle_cruise_module_param_path` 指向该文件。
   - 选择原因：RSS 目标距离包含 `v_ego × idling_time` 与 `safe_distance_margin` 两部分，仅改 `safe_distance_margin` 无法达到 3m（见 `docs/vehicle_following_scenario_task.md`）。
   - 影响范围：仅当显式覆盖 `motion_velocity_planner_obstacle_cruise_module_param_path` 时生效；默认 preset 与默认参数路径保持不变。
   - 回滚方式：不传/移除覆盖参数路径即可回到默认；或直接 `git revert` 本次提交。
3. 实施参数改动并自检：确保 YAML 语法正确、缩进保持一致；确认参数键名与作用域 `/**/ros__parameters/obstacle_cruise/cruise_planning` 一致。
   - 场景专用参数：`.../motion_velocity_planner/obstacle_cruise.bus_following_3m.param.yaml`（`idling_time=0.0`，`safe_distance_margin=3.0`），通过 `motion_velocity_planner_obstacle_cruise_module_param_path` 覆盖加载。
4. 运行连通性检查：按文档检查 dummy perception → detection → predicted objects 链路是否齐全，避免“没跟上其实是没感知输入”。
5. 运行验收验证（核心）：观察 `.../obstacle_cruise/.../planning_info`，确认 `data[7]`（目标距离）稳定接近 `3.0m` 且 `data[9]`（距离误差）接近 `0`，持续 ≥ 5 秒；同时在 RViz 观察自车轨迹与速度变化是否平滑。
6. 轻量回归与调参兜底：若出现震荡/距离不稳，优先在 `obstacle_cruise.param.yaml` 中小幅调整 `kp/kd` 与（如需要）`velocity_smoother` 配置，区分“模块内部目标距离稳定”与“最终轨迹平滑限制”两类问题。
7. 升级路径（仅在 A/B 不能满足时启用）：为 ObstacleCruise 增加 `distance_policy`（`rss|constant|time_headway`）参数，允许显式常距策略；同时补一个最小验收脚本（订阅 `planning_info` 自动判断稳定性）。
8. 交付与回退：输出本次改动的文件清单、关键参数、验证日志（topic echo 片段/截图）、以及一键回退方式（git revert 或恢复默认参数）。

⚠️ 风险与注意事项
- RSS 设计倾向时间头距，3m 属于“非常近”的跟车距离；将 `idling_time=0` 会显著降低安全裕量，默认只建议用于仿真任务验收。
- 若 perception/tracking/prediction 未启用或 Bus 被过滤/重分类，会导致跟车逻辑无法触发；需先确保 `/perception/object_recognition/objects` 中存在目标 Bus。
- `velocity_smoother` 可能掩盖 ObstacleCruise 的目标距离控制效果；验收时优先以 `planning_info` 指标为准。
- 直接修改默认配置（方案 A）会影响其它场景；如团队需要长期维护，优先采用方案 B（场景专用配置 + launch 透传）。

📎 参考
- `docs/vehicle_following_scenario_task.md:15`
- `docs/vehicle_following_scenario_task.md:22`
- `docs/vehicle_following_scenario_task.md:26`
- `docs/vehicle_following_scenario_task.md:117`
- `docs/vehicle_following_scenario_task.md:177`
- `src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_cruise.param.yaml:8`
- `src/launcher/autoware_launch/autoware_launch/launch/components/tier4_planning_component.launch.xml:94`
