# Autoware Monorepo（`autoware-mono`）协作指南（给编码代理/自动化助手）

## 项目概述

本仓库是 Autoware 自动驾驶系统 monorepo，包含完整的感知、规划、控制等模块，代码主要位于 `src/`。

## 交付原则（最小可行 & 可验证）

- 以业务价值为导向，优先提交**最小可行改动**，避免跨模块“顺手重构”。
- 每次改动都要给出**可复现的验证步骤**（本地或远端容器编译/运行），并尽量做到可追踪（关联包名/路径/配置项）。
- 优先通过检索定位（如 `rg`）锁定影响范围，再改代码；避免盲改。

## 代码结构（快速定位）

```
src/
├── core/                    # 核心库 (autoware_core)
├── launcher/                # 启动配置
│   └── autoware_launch/     # 主启动文件和参数配置
├── middleware/              # 中间件
├── sensor_component/        # 传感器组件
└── universe/                # 主要功能模块
    └── autoware_universe/   # Planning, Perception, Control 等
```

## 关键配置路径（常见改动点）

- Planning 参数：
  - `src/launcher/autoware_launch/config/planning/scenario_planning/lane_driving/`
  - 重点文件示例：
    - `.../behavior_planning/behavior_path_planner/scene_module_manager.param.yaml`
    - `.../motion_planning/motion_velocity_planner/obstacle_cruise.param.yaml`
    - `.../motion_planning/motion_velocity_planner/obstacle_stop.param.yaml`
    - `.../preset/default_preset.yaml`
- Simulation：
  - `src/universe/autoware_universe/simulator/`
  - 重点目录示例：
    - `tier4_dummy_object_rviz_plugin/`
    - `autoware_dummy_perception_publisher/`

## 开发工作流程（本地 → 推送 → 远端验证）

1. **本地**：分析、修改代码（工作目录通常为 `/opt/work/autoware-mono`）
2. **本地**：提交并推送
   - `git add -A && git commit -m "feat: description" && git push`
3. **远端**：通过 SSH MCP 在服务器容器内拉取并编译验证（推荐做法）

## 远端编译验证（通过 SSH MCP）

远端信息（来自 `CLAUDE.md`）：

- MCP 连接名：`default`
- 主容器：`heuristic_joliot`
- 镜像：`autoware:universe-devel-cuda`
- ROS：`humble`
- 远端仓库目录（Host）：`/autoware-mono`
- 容器内仓库目录（Container）：`/workspace`（与 Host 的 `/autoware-mono` 对应/挂载）

使用方式（示例）：

```
# 在服务器上执行命令
execute-command(cmdString: "命令", connectionName: "default")

# 在容器内执行命令
execute-command(cmdString: "docker exec heuristic_joliot bash -c '命令'", connectionName: "default")
```

容器内常用命令：

```bash
# 拉取最新代码（推荐：Host pull 一次即可；若未挂载则在容器内 pull）
cd /autoware-mono && git pull
# 或
docker exec heuristic_joliot bash -c "cd /workspace && git pull"

# 编译指定包
docker exec heuristic_joliot bash -c "source /opt/ros/humble/setup.bash && cd /workspace && colcon build --packages-select <package_name> --symlink-install"

# 编译全部
docker exec heuristic_joliot bash -c "source /opt/ros/humble/setup.bash && cd /workspace && colcon build --symlink-install"
```

运行 Planning Simulator（示例）：

```bash
docker exec heuristic_joliot bash -c "source /workspace/install/setup.bash && ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/path/to/map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit"
```

## 代码质量与风格（尽量遵循仓库配置）

- C/C++：遵循仓库内的 `.clang-format`、`.clang-tidy`；避免对无关文件做全量格式化。
- Python/配置：遵循 `setup.cfg`、`.yamllint.yaml`、`.markdownlint.yaml` 等已有规范。
- 只修复与当前需求直接相关的问题；如果发现相邻问题，先记录并征求是否扩展范围。

## MCP 工具使用约定（面向参数调优/场景任务）

> 适用任务示例：`docs/vehicle_following_scenario_task.md`（Bus 跟随 20km/h、稳态 3m 间距、Planning Simulator 验证）。

### 1) 项目内检索与定位（优先本地）

- 首选：`functions.shell_command` + `rg`/`rg --files` 快速定位参数键、话题名、类名、包路径；再用 `sed -n`/`nl -ba` 精读小段落。
- 辅助：`functions.list_mcp_resources` / `functions.read_mcp_resource`（若本运行环境对仓库/文档暴露了 MCP resource）。

### 2) 代码职责/链路问答（优先仓库级问答）

- 首选：`mcp__mcp-router__ask_question`（针对模块职责、参数含义、话题链路、文件入口点做“可定位到路径/符号”的提问）。
  - 示例问题模板：
    - “`autoware_motion_velocity_obstacle_cruise_module` 里 `idling_time`、`safe_distance_margin` 如何参与目标跟车距离计算？相关函数/文件在哪里？”
    - “Planning Simulator 下 DummyObject 从 RViz 工具到 `PredictedObjects` 的话题链路是什么？各节点的启动文件入口在哪？”
- 降级1：`mcp__mcp-router__read_wiki_structure` / `mcp__mcp-router__read_wiki_contents`
- 降级2：`mcp__mcp-router__fetch_content` 抓取仓库 raw 文件或 DeepWiki 页面（如网络受限需走审批/替代路径）
- 降级3：回到本地 `rg` + 逐文件阅读（必须给出路径 + 行号引用）

### 3) 外部资料检索与抓取（必要时）

- 查第三方库/API：`mcp__mcp-router__resolve-library-id` → `mcp__mcp-router__query-docs`（拿到可用的 Context7 `libraryId` 后再查示例/接口）。
- 首选：`mcp__mcp-router__search` 定位权威来源（Autoware/ROS2/消息类型/算法背景）
- 抓取：`mcp__mcp-router__fetch_content`
- 备注：若运行环境 `network_access` 受限，优先用仓库内文档/源码自证；确需联网再请求审批。

### 4) 远端容器验证（强烈推荐）

- 远端执行：`mcp__mcp-router__execute-command`（在 `heuristic_joliot` 容器内 `git pull`、`colcon build --packages-select ...`、`ros2 launch`、`ros2 topic echo`）。
- 文件传输（可选）：`mcp__mcp-router__upload` / `mcp__mcp-router__download`

### 5) 视觉/现象确认（可选）

- RViz 或日志截图需要解析时：`functions.view_image`（上传本地截图给模型做结构化观察与对照）。
