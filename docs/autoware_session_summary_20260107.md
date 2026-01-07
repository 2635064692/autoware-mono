# Autoware 研究会话总结

> 生成时间: 2026-01-07
> 环境: 远端 SSH 服务器 (117.50.152.78)
> 容器: heuristic_joliot (Autoware 主容器)

---

## 一、会话背景

本次会话围绕 Autoware 自动驾驶系统进行深入研究，主要涉及以下方面：

1. **远端环境分析** - GPU 使用情况诊断
2. **场景系统研究** - 通过 DeepWiki 获取 Autoware 支持的场景及扩展方法
3. **车辆跟随场景设计** - 实现 Bus 跟随功能的技术方案
4. **RViz 障碍物放置原理** - 深入源码分析

---

## 二、远端环境概况

### 2.1 容器状态

| 容器名 | 镜像 | 用途 | GPU |
|--------|------|------|-----|
| heuristic_joliot | autoware:universe-devel-cuda | Autoware 主容器 | Tesla P40 |
| openadkit-visualizer-v3 | autoware-tools:visualizer | RViz 可视化 | Tesla P40 |
| portainer | portainer-ce:latest | Docker 管理 | - |

### 2.2 GPU 使用分析结果

**结论: Autoware logging_simulator 确实使用了 GPU**

```
GPU: NVIDIA Tesla P40 (23040 MiB)
显存占用: ~470 MiB
利用率: 8%
```

**使用 GPU 的组件:**

| 组件 | GPU 库 | 用途 | 显存 |
|------|--------|------|------|
| LiDAR CenterPoint | libautoware_lidar_centerpoint_cuda_lib.so + TensorRT | 3D 目标检测 | 178 MiB |
| Traffic Light Classifier (camera6) | libautoware_tensorrt_classifier_gpu_preprocess.so | 交通灯分类 | 146 MiB |
| Traffic Light Classifier (camera7) | libautoware_tensorrt_classifier_gpu_preprocess.so | 交通灯分类 | 146 MiB |

---

## 三、Autoware 场景系统

### 3.1 当前支持的场景模块

#### 行为路径规划 (Behavior Path Planner) - 12 个模块

| 模块名 | 用途 | 优先级 Slot |
|--------|------|-------------|
| start_planner | 起步规划 | 高 |
| goal_planner | 目标规划 | 中 |
| lane_change_left / right | 变道 | 中 |
| external_request_lane_change_left / right | 外部请求变道 | 中 |
| static_obstacle_avoidance | 静态障碍物避让 | 中 |
| dynamic_obstacle_avoidance | 动态障碍物避让 | 低 |
| avoidance_by_lane_change | 变道避让 | 中 |
| side_shift | 侧向偏移 | 中 |
| bidirectional_traffic | 双向交通 | 中 |
| sampling_planner | 采样规划器 | 低 |

#### 行为速度规划 (Behavior Velocity Planner) - 12 个模块

| 模块名 | 用途 |
|--------|------|
| blind_spot | 盲区处理 |
| crosswalk | 人行横道 |
| walkway | 人行道 |
| detection_area | 检测区域 |
| intersection | 交叉路口 |
| merge_from_private | 私人道路汇入 |
| stop_line | 停止线 |
| virtual_traffic_light | 虚拟交通灯 |
| traffic_light | 交通灯 |
| occlusion_spot | 遮挡区域 |
| no_stopping_area | 禁停区域 |
| speed_bump | 减速带 |

#### 运动速度规划 (Motion Velocity Planner) - 4 个模块

| 模块名 | 用途 |
|--------|------|
| obstacle_stop | 障碍物停车 |
| obstacle_cruise | 障碍物巡航/跟随 |
| dynamic_obstacle_stop | 动态障碍物停车 |
| obstacle_slow_down | 障碍物减速 |

### 3.2 新增场景模块方法论

#### 步骤概览

```
1. 实现接口 (SceneModuleInterface / PluginModuleInterface)
2. 导出插件 (PLUGINLIB_EXPORT_CLASS)
3. 配置参数文件 (*.param.yaml)
4. 添加到场景管理器 (scene_module_manager.param.yaml)
5. 更新启动预设 (default_preset.yaml)
6. 修改启动文件 (tier4_planning_component.launch.xml)
```

#### 关键配置文件路径

```
autoware_launch/config/planning/scenario_planning/lane_driving/
├── behavior_planning/
│   └── behavior_path_planner/
│       └── scene_module_manager.param.yaml  # 路径规划场景管理
├── motion_planning/
│   └── motion_velocity_planner/
│       ├── obstacle_cruise.param.yaml       # 巡航跟随参数
│       └── obstacle_stop.param.yaml         # 停车参数
└── preset/
    └── default_preset.yaml                  # 模块启用预设
```

---

## 四、最新任务: 车辆跟随场景实现

### 4.1 需求定义

| 项目 | 要求 |
|------|------|
| 障碍物类型 | Bus (公交车) |
| 障碍物速度 | 20 km/h (5.56 m/s) |
| 跟随间距 | 3 米 |

### 4.2 技术方案

#### 方案架构

```
┌──────────────────────────────────────────────────────────────────┐
│                      车辆跟随场景架构                              │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌─────────────────┐      ┌─────────────────┐      ┌───────────┐ │
│  │ tier4_dummy_    │      │ dummy_          │      │ perception│ │
│  │ object_rviz_    │─────▶│ perception_     │─────▶│ /objects  │ │
│  │ plugin          │      │ publisher       │      │           │ │
│  └─────────────────┘      └─────────────────┘      └─────┬─────┘ │
│         │                                                 │      │
│         │ 发布 DummyObject                               ▼      │
│         │ (Bus, 5.56 m/s)                        ┌───────────┐  │
│                                                   │ obstacle_ │  │
│                                                   │ cruise_   │  │
│                                                   │ module    │  │
│                                                   │ (3m间距)   │  │
│                                                   └─────┬─────┘  │
│                                                         │        │
│                                                         ▼        │
│                                                   ┌───────────┐  │
│                                                   │ trajectory│  │
│                                                   │ (跟随轨迹) │  │
│                                                   └───────────┘  │
└──────────────────────────────────────────────────────────────────┘
```

#### 实施步骤

**Step 1: 修改跟随间距参数**

文件: `obstacle_cruise.param.yaml`

```yaml
/**:
  ros__parameters:
    obstacle_cruise:
      cruise_planning:
        safe_distance_margin: 3.0  # 修改为 3m
```

**Step 2: 通过 RViz 放置 Bus**

1. 在 RViz 工具栏选择 "2D Dummy Bus"
2. 在 Tool Properties 中设置 `velocity_: 5.56` (m/s)
3. 在地图上自车前方点击放置

**Step 3: 设置目标点**

使用 "2D Goal Pose" 在 Bus 前方设置目标点，自车将自动跟随

### 4.3 关键参数

| 参数 | 文件 | 值 | 说明 |
|------|------|-----|------|
| safe_distance_margin | obstacle_cruise.param.yaml | 3.0 | 跟随间距 (m) |
| velocity_ | RViz Tool Properties | 5.56 | Bus 速度 (m/s) |
| bus (inside) | obstacle_cruise.param.yaml | true | 识别 Bus |
| min_cruise_target_vel | obstacle_cruise.param.yaml | 0.0 | 最低巡航速度 |

---

## 五、RViz 手动放置障碍物原理

### 5.1 组件架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                    RViz 障碍物放置系统架构                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │                         RViz2                                  │ │
│  │  ┌──────────────────────────────────────────────────────────┐  │ │
│  │  │  tier4_dummy_object_rviz_plugin                          │  │ │
│  │  │  ├── CarInitialPoseTool      (快捷键: k)                 │  │ │
│  │  │  ├── BusInitialPoseTool      (快捷键: b)                 │  │ │
│  │  │  ├── BikeInitialPoseTool     (快捷键: c)                 │  │ │
│  │  │  ├── PedestrianInitialPoseTool                           │  │ │
│  │  │  └── DeleteAllObjectsTool                                │  │ │
│  │  └──────────────────────────────────────────────────────────┘  │ │
│  └────────────────────────────────────────────────────────────────┘ │
│                              │                                       │
│                              │ 发布 DummyObject 消息                  │
│                              ▼                                       │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │  Topic: /simulation/dummy_perception_publisher/object_info     │ │
│  │  Type:  tier4_simulation_msgs/msg/DummyObject                  │ │
│  └────────────────────────────────────────────────────────────────┘ │
│                              │                                       │
│                              ▼                                       │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │  autoware_dummy_perception_publisher                           │ │
│  │  ├── 管理障碍物状态 (ADD/MODIFY/DELETE)                        │ │
│  │  ├── 运动插件 (直线/预测轨迹)                                   │ │
│  │  ├── 生成模拟点云                                               │ │
│  │  └── 输出感知结果                                               │ │
│  └────────────────────────────────────────────────────────────────┘ │
│                              │                                       │
│            ┌─────────────────┼─────────────────┐                    │
│            ▼                 ▼                 ▼                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────────┐  │
│  │ /perception/ │  │ /sensing/    │  │ /perception/             │  │
│  │ object_      │  │ lidar/       │  │ traffic_light_           │  │
│  │ recognition/ │  │ points_raw   │  │ recognition/...          │  │
│  │ objects      │  │ (模拟点云)    │  │                          │  │
│  └──────────────┘  └──────────────┘  └──────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.2 核心流程

```
1. 用户点击 ──▶ RViz 捕获鼠标事件
                    │
2. 射线投射 ──▶ 从相机发射射线与地面求交，获取 3D 坐标
                    │
3. 创建消息 ──▶ createObjectMsg() 填充 DummyObject:
                    │   - UUID (随机生成)
                    │   - classification (BUS/CAR/...)
                    │   - shape (尺寸)
                    │   - initial_state (位置/速度)
                    │
4. 发布消息 ──▶ Topic: /simulation/dummy_perception_publisher/object_info
                    │
5. 状态管理 ──▶ DummyPerceptionPublisherNode::objectCallback()
                    │   - ADD: 添加新障碍物
                    │   - MODIFY: 更新位置/速度
                    │   - DELETE: 删除障碍物
                    │
6. 运动模拟 ──▶ MovementPlugin 更新障碍物位置
                    │   - StraightLineObjectMovementPlugin (直线运动)
                    │   - PredictedObjectMovementPlugin (预测轨迹)
                    │
7. 输出结果 ──▶ 发布到 /perception/object_recognition/objects
```

### 5.3 消息结构

```
tier4_simulation_msgs/msg/DummyObject
├── header                    # 时间戳和坐标系
├── id (UUID)                 # 唯一标识符
├── action                    # ADD=0, MODIFY=1, DELETE=2, DELETEALL=3
├── initial_state
│   ├── pose_covariance       # 位置 + 协方差
│   ├── twist_covariance      # 速度 (linear.x = 前进速度)
│   └── accel_covariance      # 加速度
├── classification            # 类型: BUS, CAR, TRUCK, PEDESTRIAN...
├── shape                     # BOUNDING_BOX + dimensions (x,y,z)
├── max_velocity              # 最大速度限制
└── min_velocity              # 最小速度限制
```

### 5.4 关键源码位置

| 文件 | 路径 | 作用 |
|------|------|------|
| car_pose.cpp | simulator/tier4_dummy_object_rviz_plugin/src/tools/ | RViz 工具实现 |
| interactive_object.cpp | 同上 | 交互式操作逻辑 |
| node.cpp | simulator/autoware_dummy_perception_publisher/src/ | 障碍物状态管理 |
| straight_line_object_movement_plugin.cpp | 同上 | 直线运动插件 |
| predicted_object_movement_plugin.cpp | 同上 | 预测轨迹运动插件 |

---

## 六、后续任务建议

1. **验证跟随场景** - 在 Planning Simulator 中测试 Bus 跟随功能
2. **参数调优** - 根据实际效果调整 `safe_distance_margin` 和 PID 参数
3. **自动化测试** - 使用 Python 脚本创建可重复的测试场景
4. **OpenSCENARIO 集成** - 将场景转换为标准格式用于回归测试

---

## 七、参考资源

| 资源 | URL |
|------|-----|
| DeepWiki - Autoware | https://deepwiki.com/autowarefoundation/autoware |
| DeepWiki - autoware_core | https://deepwiki.com/autowarefoundation/autoware_core |
| DeepWiki - autoware_launch | https://deepwiki.com/autowarefoundation/autoware_launch |
| 源码阅读目标文档 | /opt/autoware-remote/autoware_ecosystem_source_reading_goals_20260103.md |

---

*本文档由 Claude Code + Codex MCP + DeepWiki MCP 自动生成*
