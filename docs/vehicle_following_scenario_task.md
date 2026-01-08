# 车辆跟随场景实现任务文档

> 生成时间: 2026-01-07
> 基于: DeepWiki + Codex MCP 协作分析
> 目标: Bus 跟随 (20 km/h, 3m 间距)

---

## 一、任务概述

### 1.1 需求定义

| 项目 | 要求 | 说明 |
|------|------|------|
| 障碍物类型 | Bus (公交车) | ObjectClassification::BUS |
| 障碍物速度 | 20 km/h (5.56 m/s) | 在 RViz 中设置 velocity_ 属性 |
| 跟随间距 | 3 米 | 稳态目标距离 |
| 验证环境 | Planning Simulator | 无需真实传感器 |

### 1.2 关键发现

> ⚠️ **重要**: 仅修改 `safe_distance_margin` 无法实现 3m 跟随！

**原因分析:**

obstacle_cruise 模块使用 RSS (Responsibility Sensitive Safety) 公式计算目标跟车距离：

```
d_rss = v_ego × idling_time + v_ego²/(2|a_ego|) - v_obj²/(2|a_obj|) + margin
```

以 20 km/h (5.56 m/s) 匀速跟随为例：
- 默认 `idling_time = 2.0s`
- 默认 `safe_distance_margin = 5.0m`
- 计算: `d_rss ≈ 5.56 × 2.0 + 5.0 = 16.12m`

**解决方案**: 同时将 `idling_time` 降至 0，让公式退化为近似常距模式。

---

## 二、系统架构

### 2.1 完整消息流

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         车辆跟随场景完整链路                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────────┐                                                     │
│  │     RViz2           │                                                     │
│  │  BusInitialPoseTool │   Topic: /simulation/dummy_perception_publisher/    │
│  │  (velocity=5.56)    │ ──────────────────object_info────────────────────┐  │
│  └─────────────────────┘                                                  │  │
│                                                                           ▼  │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │  autoware_dummy_perception_publisher                                    │ │
│  │  ├── 管理 DummyObject 状态 (ADD/MODIFY/DELETE)                          │ │
│  │  ├── 运动插件更新位置 (StraightLine/PredictedObject)                     │ │
│  │  └── 输出: /perception/object_recognition/detection/labeled_clusters    │ │
│  └──────────────────────────────────┬──────────────────────────────────────┘ │
│                                     │                                        │
│                                     ▼                                        │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │  detected_object_feature_remover                                        │ │
│  │  └── 输出: /perception/object_recognition/detection/objects             │ │
│  └──────────────────────────────────┬──────────────────────────────────────┘ │
│                                     │                                        │
│                                     ▼                                        │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │  autoware_multi_object_tracker                                          │ │
│  │  └── 输出: /perception/object_recognition/tracking/objects              │ │
│  └──────────────────────────────────┬──────────────────────────────────────┘ │
│                                     │                                        │
│                                     ▼                                        │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │  map_based_prediction                                                   │ │
│  │  └── 输出: /perception/object_recognition/objects (PredictedObjects)    │ │
│  └──────────────────────────────────┬──────────────────────────────────────┘ │
│                                     │                                        │
│                                     ▼                                        │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │  motion_velocity_planner (MotionVelocityPlannerNode)                    │ │
│  │  ├── PlannerData 聚合输入 (odometry, predicted_objects, map...)         │ │
│  │  ├── ObstacleCruiseModule.plan()                                        │ │
│  │  │   ├── filter_cruise_obstacle_for_predicted_object()                  │ │
│  │  │   ├── PIDBasedPlanner.plan_cruise()                                  │ │
│  │  │   │   ├── calc_obstacle_to_cruise() → CruiseObstacleInfo            │ │
│  │  │   │   ├── 计算 d_rss = v*idling_time + margin (简化后)               │ │
│  │  │   │   └── PID 控制 → VelocityLimit                                  │ │
│  │  │   └── 输出: VelocityPlanningResult                                   │ │
│  │  └── insert_slowdown() / insert_stop()                                  │ │
│  └──────────────────────────────────┬──────────────────────────────────────┘ │
│                                     │                                        │
│                                     ▼                                        │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │  velocity_smoother                                                      │ │
│  │  └── 输出: /planning/scenario_planning/trajectory                       │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 关键组件源码位置

| 组件 | 源码路径 | 关键文件 |
|------|----------|----------|
| **ObstacleCruiseModule** | `src/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/` | `obstacle_cruise_module.hpp:42` |
| **PIDBasedPlanner** | 同上 `/src/pid_based_planner/` | `pid_based_planner.hpp:36` |
| **RSS 距离计算** | 同上 | `cruise_planner_interface.hpp:91-94` |
| **DummyPerceptionPublisher** | `src/universe/autoware_universe/simulator/autoware_dummy_perception_publisher/` | `node.hpp:102` |
| **BusInitialPoseTool** | `src/universe/autoware_universe/simulator/tier4_dummy_object_rviz_plugin/src/tools/` | `car_pose.hpp:68` |
| **MapBasedPrediction** | `src/universe/autoware_universe/perception/autoware_map_based_prediction/` | `map_based_prediction_node.cpp:490` |

---

## 三、实施计划

### 3.1 配置（方案 B：场景专用参数 + launch 覆盖）

为避免污染默认配置，本仓库新增了 Bus 跟随 3m 的场景专用参数文件，并通过 launch 参数覆盖加载：

- 场景参数文件：`.../motion_velocity_planner/obstacle_cruise.bus_following_3m.param.yaml`
- 覆盖的关键参数：
  - `cruise_planning.idling_time = 0.0`
  - `cruise_planning.safe_distance_margin = 3.0`
- 透传/覆盖用的 launch arg：`motion_velocity_planner_obstacle_cruise_module_param_path`

**使用方式：**

```bash
# 3m 场景：显式覆盖 obstacle_cruise 参数文件路径
ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path:=/path/to/map \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit \
    motion_velocity_planner_obstacle_cruise_module_param_path:=$(ros2 pkg prefix --share autoware_launch)/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_cruise.bus_following_3m.param.yaml
```

> 基线（默认配置）采集：不要传 `motion_velocity_planner_obstacle_cruise_module_param_path`，保持默认参数文件路径不变。

### 3.2 RViz 操作步骤

1. **启动 Planning Simulator**
   ```bash
   ros2 launch autoware_launch planning_simulator.launch.xml \
       map_path:=/path/to/map \
       vehicle_model:=sample_vehicle \
       sensor_model:=sample_sensor_kit
   ```

2. **放置 Bus 障碍物**
   - 在 RViz 工具栏选择 "2D Dummy Bus" (快捷键: `b`)
   - 在 Tool Properties 面板设置:
     - `Velocity`: `5.56` (m/s = 20 km/h)
     - `Pose Topic`: `/simulation/dummy_perception_publisher/object_info` (默认)
   - 在自车前方同车道点击放置 (横向偏差 < 1m)

3. **设置目标点**
   - 使用 "2D Goal Pose" 在 Bus 前方设置目标点
   - 目标入口（手动与自动应复用同一入口）：
     - **Topic**：`/planning/mission_planning/goal`
     - **Type**：`geometry_msgs/msg/PoseStamped`
     - **QoS（RViz 默认配置）**：Reliable + Volatile，Depth=5（见 `src/launcher/autoware_launch/autoware_launch/rviz/autoware.rviz:4187`）
     - 该 topic 由 `autoware_adapi_adaptors` 的 `routing_adaptor_node` 订阅，并转换为路由 AD API 的 service 调用（无需改动 mission_planner）
     - **Service**（路由 AD API）：`/api/routing/set_route_points`（`autoware_adapi_v1_msgs/srv/SetRoutePoints`）、`/api/routing/clear_route`（`autoware_adapi_v1_msgs/srv/ClearRoute`）
   - 自动发布建议：
     - 发布频率建议 `<= 2Hz`，并配合位置/角度阈值做节流，避免触发频繁重规划
     - **互斥策略**：启用自动跟随（`enable_auto_follow=true`）时不要再使用 RViz 的 "2D Goal Pose"；如需手动 goal，请关闭自动跟随（否则自动节点会持续更新并覆盖手动 goal，并输出告警日志）
   - 自车将自动规划并跟随 Bus

### 3.3 验证步骤

**A. 链路连通性检查**

```bash
# 0. 确认 RViz 目标入口与消息类型
ros2 topic info -v /planning/mission_planning/goal

# 0.1 确认 route 设置链路的 service 类型（routing_adaptor 会调用这些 API）
ros2 service type /api/routing/set_route_points
ros2 service type /api/routing/clear_route

# 1. 检查 DummyObject 是否发布
ros2 topic echo /simulation/dummy_perception_publisher/object_info --once

# 2. 检查 Detection 输出
ros2 topic echo /perception/object_recognition/detection/objects --once

# 3. 检查 PredictedObjects (planning 输入)
ros2 topic echo /perception/object_recognition/objects --once
```

**常见缺失定位（最小可行）**

- 三个话题都无输出：优先确认 RViz 放置工具的 `Pose Topic` 为 `/simulation/dummy_perception_publisher/object_info`，以及 `planning_simulator.launch.xml` 中 `perception/enable_object_recognition=true`（默认即为 true）。
- 仅 `object_info` 有输出：说明 DummyObject 已发布，下一步用 `ros2 topic info -v /perception/object_recognition/detection/objects` 确认是否有 publisher；若无，检查 dummy perception 相关节点是否启动。
- `detection/objects` 有输出但 `object_recognition/objects` 无输出：优先检查 prediction/objects 相关节点是否启动、以及 Bus 分类是否为 `BUS`（被过滤/重分类会导致不进入 planning 输入）。

**B. ObstacleCruise 模块状态**

```bash
# 查看相关话题
ros2 topic list | grep obstacle_cruise

# 观察 debug 信息
ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/obstacle_cruise/debug/obstacle_cruise/planning_info
```

**C. 验收指标 (planning_info)**

`planning_info` 消息类型: `tier4_debug_msgs/msg/Float32MultiArrayStamped`

关键指标索引 (定义于 `cruise_planning_debug_info.hpp:31`):

| Index | 名称 | 验收标准 |
|-------|------|----------|
| 7 | `CRUISE_TARGET_OBSTACLE_DISTANCE` | 趋近 3.0 m |
| 4 | `CRUISE_CURRENT_OBSTACLE_DISTANCE_FILTERED` | 实际距离估计 |
| 9 | `CRUISE_ERROR_DISTANCE_FILTERED` | 趋近 0 |

**验收口径**: `data[7] ≈ 3.0` 且 `data[9] ≈ 0` 持续 5 秒以上。

**取证建议（推荐其一）：**

```bash
# 记录一段连续输出用于 Review（再手动标注满足 ≥5s 的区间）
timeout 10s ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/obstacle_cruise/debug/obstacle_cruise/planning_info \\
  | tee phase5_planning_info.log
```

---

### 3.4 Phase 1 基线采集（默认配置）

> 目的：拿到“未改参数前”的对照基线，便于后续验证 3m 参数是否生效、以及 Phase 6 调参时做前后对比。

**前置条件：**
- 启动 `planning_simulator.launch.xml` 时**不要**覆盖 `motion_velocity_planner_obstacle_cruise_module_param_path`（保持默认参数文件）。
- RViz 放置 Bus（`Velocity=5.56`）后，再开始采集 `planning_info`。

**采集命令（任选其一）：**

```bash
# 只抓取 1 帧并保存原始输出，作为可追溯证据
ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/obstacle_cruise/debug/obstacle_cruise/planning_info --once \\
  | tee baseline_planning_info.txt
```

**记录项（手动填入 PR/提交说明或验收记录）：**
- 使用的 `map_path / vehicle_model / sensor_model`
- `planning_info.data[7]`（目标距离）与 `planning_info.data[9]`（距离误差）的截图/日志
- 启动命令（含所有 launch args）

### 3.5 交付、回滚与可追溯性（可直接复制到 PR/提交说明）

**改动文件列表（关键行为变更）：**
- `src/launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml`：新增并透传 `motion_velocity_planner_obstacle_cruise_module_param_path`
- `src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml`：透传 `motion_velocity_planner_obstacle_cruise_module_param_path` 到 planning 组件
- `src/launcher/autoware_launch/autoware_launch/launch/components/tier4_planning_component.launch.xml`：允许覆盖 `motion_velocity_planner_obstacle_cruise_module_param_path`
- `src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_cruise.bus_following_3m.param.yaml`：场景专用参数（`idling_time=0.0`，`safe_distance_margin=3.0`）

**关键参数值：**
- `cruise_planning.idling_time = 0.0`
- `cruise_planning.safe_distance_margin = 3.0`
- 覆盖入口：launch arg `motion_velocity_planner_obstacle_cruise_module_param_path`（见 “配置（方案 B）”）

**Phase 5 验收证据（待补，建议产物）：**
- `phase5_planning_info.log`（见 “取证建议” 的 `timeout 10s ... | tee`）
- RViz 截图/录屏（展示稳定跟随与无明显震荡）

**一键回滚：**
- 运行时回滚：不传 `motion_velocity_planner_obstacle_cruise_module_param_path`（恢复默认参数文件路径）。
- 代码回滚（按需）：`git revert b8f3dca 0605da9`（先回滚 `b8f3dca` 再回滚 `0605da9`）。

## 四、关键参数说明

### 4.1 obstacle_cruise.param.yaml 完整参数

**巡航规划参数 (cruise_planning):**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `idling_time` | 2.0 | 反应时间，影响 RSS 距离 [s] |
| `min_ego_accel_for_rss` | -1.0 | 自车制动加速度 [m/s²] |
| `min_object_accel_for_rss` | -1.0 | 前车制动加速度 [m/s²] |
| `safe_distance_margin` | 5.0 | 安全距离裕量 [m] |
| `min_accel_during_cruise` | -2.0 | 最小巡航减速度 [m/s²] |
| `min_cruise_target_vel` | 0.0 | 最小目标速度 [m/s] |

**PID 控制参数 (velocity_limit_based_planner):**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `kp` | 10.0 | 比例增益 |
| `ki` | 0.0 | 积分增益 |
| `kd` | 2.0 | 微分增益 |
| `output_ratio_during_accel` | 0.6 | 加速时输出比例 |

**障碍物过滤参数 (obstacle_filtering):**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `object_type.inside.bus` | true | 识别轨迹内的 Bus |
| `object_type.outside.bus` | true | 识别轨迹外的 Bus |
| `max_lat_margin` | 1.0 | 最大横向距离 [m] |
| `obstacle_velocity_threshold_from_cruise` | 3.0 | 巡航转停车速度阈值 [m/s] |

### 4.2 DummyObject 消息结构

```
tier4_simulation_msgs/msg/DummyObject
├── header                    # 时间戳和坐标系
├── id (UUID)                 # 唯一标识符
├── action                    # ADD=0, MODIFY=1, DELETE=2, DELETEALL=3
├── initial_state
│   ├── pose_covariance       # 位置 + 协方差
│   ├── twist_covariance      # 速度 (linear.x = 前进速度)
│   └── accel_covariance      # 加速度
├── classification            # 类型: BUS, CAR, TRUCK...
├── shape                     # BOUNDING_BOX + dimensions (x,y,z)
├── max_velocity              # 最大速度限制
└── min_velocity              # 最小速度限制
```

---

## 五、风险点与对策

### 5.1 需求与算法不匹配

| 风险 | 描述 | 对策 |
|------|------|------|
| RSS 设计目标冲突 | RSS 公式天然倾向时间头距，3m 是非常近的跟车距离 | 将 `idling_time` 降到 0 |
| 安全性降低 | 0s 反应时间意味着无安全裕量 | 仅用于仿真验证，实车需重新评估 |

### 5.2 感知链路问题

| 风险 | 描述 | 对策 |
|------|------|------|
| PredictedObjects 缺失 | tracking/prediction 未启动 | 确认 `perception/enable_object_recognition: true` |
| 目标车被过滤 | 横向偏差过大或姿态不符 | 放置时保证 Bus 与车道方向一致 |
| 类型被改写 | tracker 重新分类 | 检查 `/perception/object_recognition/objects` 中的 classification |

### 5.3 控制层面

| 风险 | 描述 | 对策 |
|------|------|------|
| 目标距离不稳定 | velocity_smoother 限制 | 检查 ObstacleCruise 自身指标，与最终轨迹分开验证 |
| PID 震荡 | 参数不匹配 | 调整 kp/kd 或启用低通滤波 |

**Phase 6 兜底调参（最小边界 + 可追溯取证）**

> 原则：只在“场景专用参数文件”内做最小调参；每次只动 1–2 个参数；每次都必须留下前后对照证据。

1. **先判定是哪一层导致“震荡/不稳”**
   - 若 `planning_info.data[7]/data[9]` 已稳定，但 RViz 轨迹仍抖：优先排查 `velocity_smoother` 的平滑/限幅影响（不要先动 ObstacleCruise）。
   - 若 `planning_info.data[7]/data[9]` 本身不稳定：再考虑调 ObstacleCruise 的 PID 参数。

2. **推荐的最小调参范围（按优先级）**
   - 文件：`.../motion_velocity_planner/obstacle_cruise.bus_following_3m.param.yaml`
   - 优先只调整 `pid_based_planner.velocity_limit_based_planner`：
     - `kp`：降低可减小超调/震荡（例如每次 -10%）
     - `kd`：适当增加可抑制振荡（例如每次 +10%）

3. **取证方式（建议固定格式）**
   - 启动命令与参数文件路径（必须包含 `motion_velocity_planner_obstacle_cruise_module_param_path`）
   - `phase5_planning_info.log`（见 “取证建议” 的 `timeout 10s ... | tee`）
   - 调参前后各保留一份日志，并在文件名中写明 `kp/kd` 取值

---

## 六、扩展建议

> 说明：本节为“可选升级项”的设计与草案，用于在 Phase 5/6 无法稳定达标时再启用；当前仓库未实现对应代码改动，落地需在 ROS2 环境编译验证。

### 6.1 增加距离策略参数 (推荐)

在 ObstacleCruiseModule 中增加可选的距离计算策略：

```yaml
obstacle_cruise:
  cruise_planning:
    distance_policy: "constant"  # rss | constant | time_headway
    constant_distance: 3.0       # 当 policy=constant 时使用
    time_headway: 0.6            # 当 policy=time_headway 时使用
```

**实现位置**: `pid_based_planner.cpp:227` (计算 `target_dist_to_obstacle` 前)

### 6.2 场景包化

创建可复用的场景启动文件：

```xml
<!-- vehicle_following_bus_20kph_3m.launch.xml -->
<launch>
  <arg name="bus_velocity" default="5.56"/>
  <arg name="following_distance" default="3.0"/>

  <!-- Override obstacle_cruise params -->
  <arg name="motion_velocity_planner_obstacle_cruise_module_param_path"
       default="$(find-pkg-share my_scenario_pkg)/config/obstacle_cruise_follow_bus.param.yaml"/>

  <!-- Include planning_simulator with overrides -->
  <include file="$(find-pkg-share autoware_launch)/launch/planning_simulator.launch.xml">
    <arg name="motion_velocity_planner_obstacle_cruise_module_param_path"
         value="$(var motion_velocity_planner_obstacle_cruise_module_param_path)"/>
  </include>
</launch>
```

### 6.3 自动化验收脚本

```python
#!/usr/bin/env python3
# verify_following_distance.py

import rclpy
from tier4_debug_msgs.msg import Float32MultiArrayStamped

TARGET_DISTANCE = 3.0
TOLERANCE = 0.5
STABLE_DURATION = 5.0  # seconds

class FollowingDistanceVerifier:
    def __init__(self):
        self.node = rclpy.create_node('following_distance_verifier')
        self.sub = self.node.create_subscription(
            Float32MultiArrayStamped,
            '/planning/.../planning_info',
            self.callback, 10)
        self.stable_start = None

    def callback(self, msg):
        target_dist = msg.data[7]  # CRUISE_TARGET_OBSTACLE_DISTANCE
        error = msg.data[9]        # CRUISE_ERROR_DISTANCE_FILTERED

        if abs(target_dist - TARGET_DISTANCE) < TOLERANCE and abs(error) < TOLERANCE:
            if self.stable_start is None:
                self.stable_start = self.node.get_clock().now()
            elif (self.node.get_clock().now() - self.stable_start).nanoseconds > STABLE_DURATION * 1e9:
                print("PASS: Following distance stable at 3m")
        else:
            self.stable_start = None
```

---

## 七、参考资源

### 7.1 源码位置索引

| 功能 | 文件路径 | 行号 |
|------|----------|------|
| RSS 距离计算 | `autoware_motion_velocity_obstacle_cruise_module/src/cruise_planner_interface.hpp` | 91-94 |
| PID 巡航规划 | `autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` | 227 |
| 障碍物过滤 | `autoware_motion_velocity_obstacle_cruise_module/src/obstacle_cruise_module.cpp` | - |
| Debug 指标定义 | `autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/cruise_planning_debug_info.hpp` | 31 |
| Bus 放置工具 | `tier4_dummy_object_rviz_plugin/src/tools/car_pose.cpp` | 169, 190 |
| Velocity 设置 | `tier4_dummy_object_rviz_plugin/src/tools/interactive_object.cpp` | 285, 290 |
| PredictedObjects 发布 | `autoware_map_based_prediction/src/map_based_prediction_node.cpp` | 490 |

### 7.2 启动文件链路

```
planning_simulator.launch.xml:96
  └── tier4_simulator_component.launch.xml:20
        └── simulator.launch.xml:63
              ├── dummy_perception_publisher.launch.xml:14 (detection)
              ├── :97 autoware_multi_object_tracker (tracking)
              └── :109 prediction.launch.xml → map_based_prediction
```

### 7.3 DeepWiki 文档

| 仓库 | URL |
|------|-----|
| autoware_core | https://deepwiki.com/autowarefoundation/autoware_core#4.2 |
| autoware_msgs | https://deepwiki.com/autowarefoundation/autoware_msgs#1.2 |
| autoware_launch | https://deepwiki.com/autowarefoundation/autoware_launch#3 |

---

*本文档由 Claude Code + Codex MCP + DeepWiki MCP 协作生成*
*生成时间: 2026-01-07*
