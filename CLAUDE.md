# Autoware Monorepo 开发指南

## 项目概述

Autoware 自动驾驶系统 monorepo，包含完整的感知、规划、控制模块。

## 开发工作流程

```
┌─────────────────┐     git push     ┌─────────────────┐
│   本地开发环境    │ ──────────────▶ │   GitHub Repo   │
│ /opt/work/      │                  │                 │
│ autoware-mono   │                  │                 │
└─────────────────┘                  └────────┬────────┘
                                              │
                        ┌─────────────────────┘
                        │ SSH MCP (execute-command)
                        ▼
              ┌─────────────────┐     docker exec    ┌─────────────────┐
              │   远端服务器      │ ────────────────▶ │ heuristic_joliot│
              │ (SSH MCP连接)    │                   │ 容器内编译验证    │
              └─────────────────┘                   │ (Tesla P40 GPU) │
                                                    └─────────────────┘
```

### 步骤
1. **本地**: 代码分析与修改
2. **本地**: `git add && git commit && git push`
3. **远端**: 通过 SSH MCP 执行命令，拉取代码并在容器内编译验证

## 远端环境

通过 SSH MCP 工具连接，无需固定 IP。

| 项目 | 值 |
|------|-----|
| MCP 连接名 | `default` |
| 主容器 | heuristic_joliot |
| 可视化容器 | openadkit-visualizer-v3 |
| GPU | NVIDIA Tesla P40 (23040 MiB) |
| 镜像 | autoware:universe-devel-cuda |

### SSH MCP 使用方式

使用 `execute-command` 工具执行远端命令：

```
# 在服务器上执行命令
execute-command(cmdString: "命令", connectionName: "default")

# 在容器内执行命令
execute-command(cmdString: "docker exec heuristic_joliot bash -c '命令'")
```

### 容器内编译 (通过 SSH MCP)
```bash
# 拉取最新代码
docker exec heuristic_joliot bash -c "cd /autoware && git pull"

# 编译指定包
docker exec heuristic_joliot bash -c "source /opt/ros/humble/setup.bash && cd /autoware && colcon build --packages-select <package_name> --symlink-install"

# 编译全部
docker exec heuristic_joliot bash -c "source /opt/ros/humble/setup.bash && cd /autoware && colcon build --symlink-install"
```

## 代码结构

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

## 关键配置路径

### Planning 模块
```
src/launcher/autoware_launch/config/planning/scenario_planning/lane_driving/
├── behavior_planning/
│   └── behavior_path_planner/
│       └── scene_module_manager.param.yaml    # 路径规划场景管理
├── motion_planning/
│   └── motion_velocity_planner/
│       ├── obstacle_cruise.param.yaml         # 巡航跟随参数
│       └── obstacle_stop.param.yaml           # 停车参数
└── preset/
    └── default_preset.yaml                    # 模块启用预设
```

### Simulation 模块
```
src/universe/autoware_universe/simulator/
├── tier4_dummy_object_rviz_plugin/    # RViz 障碍物放置工具
└── autoware_dummy_perception_publisher/ # 模拟感知发布器
```

## 场景模块

### 行为路径规划 (12 模块)
start_planner, goal_planner, lane_change_*, static_obstacle_avoidance,
dynamic_obstacle_avoidance, avoidance_by_lane_change, side_shift,
bidirectional_traffic, sampling_planner

### 行为速度规划 (12 模块)
blind_spot, crosswalk, walkway, detection_area, intersection,
merge_from_private, stop_line, virtual_traffic_light, traffic_light,
occlusion_spot, no_stopping_area, speed_bump

### 运动速度规划 (4 模块)
obstacle_stop, obstacle_cruise, dynamic_obstacle_stop, obstacle_slow_down

## GPU 组件

| 组件 | 显存 | 用途 |
|------|------|------|
| LiDAR CenterPoint | 178 MiB | 3D 目标检测 (TensorRT) |
| Traffic Light Classifier (×2) | 292 MiB | 交通灯分类 |

## 常用命令

### 本地 Git 操作
```bash
# 提交并推送
git add -A && git commit -m "feat: description" && git push

# 查看状态
git status && git diff --stat
```

### 远端验证 (通过 SSH MCP)

使用 `execute-command` 工具，所有命令需通过 `docker exec` 在容器内执行：

```bash
# 拉取代码
docker exec heuristic_joliot bash -c "cd /autoware && git pull"

# 编译指定包
docker exec heuristic_joliot bash -c "source /opt/ros/humble/setup.bash && cd /autoware && colcon build --packages-select <pkg> --symlink-install"

# 运行 Planning Simulator
docker exec heuristic_joliot bash -c "source /autoware/install/setup.bash && ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/path/to/map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit"
```

## 当前研究任务

- [ ] 车辆跟随场景 (Bus, 20 km/h, 3m 间距)
- [ ] RViz 障碍物放置原理分析
- [ ] 新增场景模块方法论验证

## 参考资源

- [DeepWiki - Autoware](https://deepwiki.com/autowarefoundation/autoware)
- [DeepWiki - autoware_core](https://deepwiki.com/autowarefoundation/autoware_core)
- [DeepWiki - autoware_launch](https://deepwiki.com/autowarefoundation/autoware_launch)
