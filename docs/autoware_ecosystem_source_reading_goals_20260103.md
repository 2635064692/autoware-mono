# Autoware 生态系统源码阅读目标

> 生成时间：2026-01-03
> 覆盖仓库：autowarefoundation 6 大核心仓库
> 版本：main 分支 / 最新稳定版

---

【版本与环境前置】

| 项目 | 要求 | 说明 |
|------|------|------|
| **操作系统** | Ubuntu 22.04 LTS | 推荐使用 Docker 容器 |
| **ROS 版本** | ROS 2 Humble | Humble Hawksbill |
| **硬件** | 16GB+ RAM, NVIDIA GPU (CUDA 11.x) | 感知模块需要 GPU 加速 |
| **存储** | 100GB+ 可用空间 | 源码 + 地图 + 模型 |
| **Docker 镜像** | `ghcr.io/autowarefoundation/autoware:universe-devel-cuda` | 开发环境 |

**仓库 URL 速查**

| 仓库 | GitHub URL |
|------|------------|
| autoware | https://github.com/autowarefoundation/autoware |
| autoware_core | https://github.com/autowarefoundation/autoware_core |
| autoware_universe | https://github.com/autowarefoundation/autoware.universe |
| autoware_msgs | https://github.com/autowarefoundation/autoware_msgs |
| autoware_launch | https://github.com/autowarefoundation/autoware_launch |
| autoware_lanelet2_extension | https://github.com/autowarefoundation/autoware_lanelet2_extension |

> **注意**：`autoware_core` 是 2024 年新引入的独立仓库，用于承载稳定、高质量的核心抽象接口，与 `autoware.universe`（实验性功能）形成分层。部分组件正从 universe 向 core 迁移中。

---

【结论与价值】

- 本次阅读计划面向「Autoware 自动驾驶生态系统 @main」，优先解决：**理解多仓库架构边界与依赖关系、掌握感知-规划-控制全链路数据流、深入核心抽象接口与实现模式**。
- 产出：可执行的跨仓库阅读路线 + 双验证场景（规划仿真 + 日志回放）；预期收获：
  - 自动驾驶系统架构清晰度
  - Core/Universe 边界与演进策略
  - 消息契约与模块解耦机制
  - 实车部署与仿真验证路径

---

【仓库职责速查】

| 仓库 | 类型 | 主要职责 | 关键技术 |
|------|------|----------|----------|
| **autoware** | 元仓库 | 依赖管理、Docker 构建、CI/CD | vcs, Docker, GitHub Actions |
| **autoware_core** | 核心 | 抽象接口、稳定组件、5层架构 | ROS 2, C++, Lanelet2 |
| **autoware_universe** | 功能 | 实验性功能实现 | 深度学习, PCL, OpenCV, TensorRT |
| **autoware_msgs** | 接口 | 消息/服务定义 | ROS 2 IDL |
| **autoware_launch** | 配置 | 启动文件、参数管理、4种运行模式 | ROS 2 Launch, YAML |
| **autoware_lanelet2_extension** | 地图 | Lanelet2 格式扩展、监管元素 | Lanelet2, C++ |

---

【主线子系统（11条，混合划分：功能链路 × 仓库映射）】

## 1. 架构与依赖编排

- **为什么重要**：理解多仓库协作的构建链路与版本锁定策略，是后续源码阅读的基础设施保障。

- **源码入口**：
  - 元仓库入口：[autoware.repos](https://github.com/autowarefoundation/autoware/blob/main/autoware.repos)
  - 夜间构建：[autoware-nightly.repos](https://github.com/autowarefoundation/autoware/blob/main/autoware-nightly.repos)
  - Docker 构建：[docker/Dockerfile](https://github.com/autowarefoundation/autoware/tree/main/docker)
  - CI/CD：[.github/workflows/docker-build-and-push.yaml](https://github.com/autowarefoundation/autoware/blob/main/.github/workflows/docker-build-and-push.yaml)
  - 开发环境配置：[setup-dev-env.sh](https://github.com/autowarefoundation/autoware/blob/main/setup-dev-env.sh)
  - 相关文档：https://deepwiki.com/autowarefoundation/autoware#1

- **预期洞察**：
  1. 理解 `.repos` 文件如何通过 `vcs import` 管理多仓库依赖版本
  2. 掌握 Docker 镜像层次（core-devel → universe-devel → universe-cuda）
  3. 学习 CI/CD 流水线的自动化构建与发布策略

- **验证方式**：
  ```bash
  # 克隆元仓库并导入依赖
  git clone https://github.com/autowarefoundation/autoware.git
  cd autoware
  vcs import src < autoware.repos

  # 验证构建环境
  docker run -it --rm ghcr.io/autowarefoundation/autoware:universe-devel-cuda
  ```

---

## 2. 消息与接口契约

- **为什么重要**：消息定义是模块解耦的核心契约，决定跨模块通信与演进成本。

- **源码入口**：
  - 感知消息：[autoware_perception_msgs](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs)
  - 规划消息：[autoware_planning_msgs](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_planning_msgs)
  - 控制消息：[autoware_control_msgs](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_control_msgs)
  - 车辆消息：[autoware_vehicle_msgs](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_vehicle_msgs)
  - 地图消息：[autoware_map_msgs](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs)
  - 系统消息：[autoware_system_msgs](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_system_msgs)
  - 接口规范（core）：[autoware_component_interface_specs](https://github.com/autowarefoundation/autoware_core/tree/main/common/autoware_component_interface_specs)
  - 相关文档：https://deepwiki.com/autowarefoundation/autoware_msgs#1

- **预期洞察**：
  1. 理解 `DetectedObjects → TrackedObjects → PredictedObjects` 数据流转
  2. 掌握 `Trajectory` 消息如何表示时空轨迹点序列
  3. 学习 `autoware_component_interface_specs` 中的话题/QoS 标准化定义

- **验证方式**：
  ```bash
  # 查看消息定义
  ros2 interface show autoware_perception_msgs/msg/DetectedObjects
  ros2 interface show autoware_planning_msgs/msg/Trajectory

  # 导出主题列表快照用于回归
  ros2 topic list | sort > topic_snapshot.txt
  ```

---

## 3. 启动编排与运行模式

- **为什么重要**：理解 4 种运行模式的差异与参数传递链，是验证与调试的关键。

- **源码入口**：
  - 主入口：[autoware.launch.xml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/autoware.launch.xml)
  - 规划仿真：[planning_simulator.launch.xml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/planning_simulator.launch.xml)
  - E2E 仿真：[e2e_simulator.launch.xml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/e2e_simulator.launch.xml)
  - 日志回放：[logging_simulator.launch.xml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/logging_simulator.launch.xml)
  - 组件启动：[components/](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch/launch/components)
  - 相关文档：https://deepwiki.com/autowarefoundation/autoware_launch#2

- **预期洞察**：
  1. 理解 4 种模式如何通过参数覆盖禁用/启用组件
  2. 掌握 `autoware_global_parameter_loader` 的参数分发机制
  3. 学习 `tier4_*_launch` 组件启动文件的层次结构

- **验证方式**：
  ```bash
  # 规划仿真模式
  ros2 launch autoware_launch planning_simulator.launch.xml \
      map_path:=$HOME/autoware_map/sample-map-planning \
      vehicle_model:=sample_vehicle \
      sensor_model:=sample_sensor_kit

  # 日志回放模式
  ros2 launch autoware_launch logging_simulator.launch.xml
  ```

---

## 4. 核心抽象层 - autoware_core 架构

- **为什么重要**：autoware_core 提供稳定、高质量的基础组件和抽象接口，是功能模块的基石。

- **源码入口**：
  - 系统入口：[autoware_core.launch.xml](https://github.com/autowarefoundation/autoware_core/blob/main/autoware_core/launch/autoware_core.launch.xml)
  - 轨迹模板库：[autoware_trajectory](https://github.com/autowarefoundation/autoware_core/tree/main/common/autoware_trajectory)
  - 地图工具：[autoware_lanelet2_utils](https://github.com/autowarefoundation/autoware_core/tree/main/common/autoware_lanelet2_utils)
  - 路由处理：[autoware_route_handler](https://github.com/autowarefoundation/autoware_core/tree/main/planning/autoware_route_handler)
  - 速度规划器：[autoware_motion_velocity_planner](https://github.com/autowarefoundation/autoware_core/tree/main/planning/autoware_motion_velocity_planner)
  - 插件接口：`planning/autoware_motion_velocity_planner/include/.../plugin_module_interface.hpp`
  - 相关文档：https://deepwiki.com/autowarefoundation/autoware_core#1

- **预期洞察**：
  1. 理解 5 层架构：开发基础设施 → 地图路线 → 规划核心 → 速度模块 → 通用库
  2. 掌握 `PluginModuleInterface` 插件架构的设计模式
  3. 学习 `PlannerData` 共享数据结构的实现

- **验证方式**：
  ```bash
  # 运行 autoware_core 测试
  colcon test --packages-select autoware_trajectory autoware_route_handler
  ```

---

## 5. 定位系统 - NDT 匹配与 EKF 融合

- **为什么重要**：高精度定位是自动驾驶的基石，决定车辆在地图中的精确位置。

- **源码入口**：
  - NDT 扫描匹配：[localization/ndt_scan_matcher](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/ndt_scan_matcher)
  - EKF 定位融合（core）：[autoware_ekf_localizer](https://github.com/autowarefoundation/autoware_core/tree/main/localization/autoware_ekf_localizer)
  - 位姿初始化：[localization/pose_initializer](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/pose_initializer)
  - 定位监控：[localization/localization_error_monitor](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/localization_error_monitor)
  - 启动文件：[tier4_localization_component.launch.xml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/components/tier4_localization_component.launch.xml)

- **预期洞察**：
  1. 理解 NDT（Normal Distributions Transform）算法的迭代收敛过程
  2. 掌握 EKF 如何融合 GNSS、IMU、轮速计与 NDT 输出
  3. 学习定位误差监控与故障检测机制

- **验证方式**：
  ```bash
  # 查看定位输出
  ros2 topic echo /localization/pose_estimator/pose --once
  ros2 topic echo /localization/pose_estimator/pose_with_covariance --once
  ```

---

## 6. 地图系统 - Lanelet2 扩展

- **为什么重要**：高精地图提供语义信息，监管元素直接影响规划决策。

- **源码入口**：
  - 监管元素定义：[regulatory_elements/](https://github.com/autowarefoundation/autoware_lanelet2_extension/tree/main/autoware_lanelet2_extension/lib/regulatory_elements)
  - 交通灯扩展：[autoware_traffic_light.cpp](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/lib/regulatory_elements/autoware_traffic_light.cpp)
  - 检测区域：[detection_area.cpp](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/lib/regulatory_elements/detection_area.cpp)
  - 人行横道：[crosswalk.cpp](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/lib/regulatory_elements/crosswalk.cpp)
  - 查询 API：[query.cpp](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/lib/query.cpp)
  - 可视化 API：[visualization.cpp](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/lib/visualization.cpp)
  - 相关文档：https://deepwiki.com/autowarefoundation/autoware_lanelet2_extension#4

- **预期洞察**：
  1. 理解 `AutowareTrafficLight`、`DetectionArea`、`Crosswalk` 等监管元素的定义
  2. 掌握车道子类型（road_shoulder、pedestrian_lane、bicycle_lane）的语义
  3. 学习 `lanelet::utils::query` 命名空间的空间查询 API

- **验证方式**：
  ```bash
  # 在 RViz2 中可视化地图
  ros2 launch autoware_launch planning_simulator.launch.xml map_path:=<path>
  # 观察地图标记与监管元素可视化
  ```

---

## 7. 感知系统 - 物体检测与跟踪

- **为什么重要**：感知是自动驾驶的"眼睛"，直接决定环境理解的准确性。

- **源码入口**：
  - CenterPoint 检测：[perception/lidar_centerpoint](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/lidar_centerpoint)
  - BEVFusion 融合：[perception/bevfusion](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/bevfusion)
  - 地面分割：[perception/ground_segmentation](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/ground_segmentation)
  - 多目标跟踪：[perception/multi_object_tracker](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/multi_object_tracker)
  - 轨迹预测：[perception/map_based_prediction](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/map_based_prediction)
  - 交通灯识别：[perception/traffic_light_recognition](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/traffic_light_recognition)
  - 启动文件：[tier4_perception_component.launch.xml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/components/tier4_perception_component.launch.xml)
  - 相关文档：https://deepwiki.com/autowarefoundation/autoware_universe#3

- **预期洞察**：
  1. 理解 LiDAR 点云如何通过 CenterPoint 转换为 3D 物体检测结果
  2. 掌握 EKF/UKF 在多目标跟踪中的状态估计与数据关联
  3. 学习 TensorRT 推理加速策略

- **验证方式**：
  ```bash
  colcon test --packages-select autoware_lidar_centerpoint autoware_multi_object_tracker
  ros2 topic echo /perception/object_recognition/detection/objects --once
  ros2 topic echo /perception/object_recognition/prediction/objects --once
  ```

---

## 8. 规划系统 - 行为与运动规划

- **为什么重要**：规划是决策层核心，将感知结果转化为可执行轨迹。

- **源码入口**：
  - 行为路径规划：[planning/behavior_path_planner](https://github.com/autowarefoundation/autoware.universe/tree/main/planning/behavior_path_planner)
  - 场景模块：[planning/behavior_path_planner_common](https://github.com/autowarefoundation/autoware.universe/tree/main/planning/behavior_path_planner_common)
  - 行为速度规划：[planning/behavior_velocity_planner](https://github.com/autowarefoundation/autoware.universe/tree/main/planning/behavior_velocity_planner)
  - 运动规划：[planning/motion_velocity_planner](https://github.com/autowarefoundation/autoware.universe/tree/main/planning/motion_velocity_planner)
  - 路径生成器（core）：[autoware_path_generator](https://github.com/autowarefoundation/autoware_core/tree/main/planning/autoware_path_generator)
  - 启动文件：[tier4_planning_component.launch.xml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/components/tier4_planning_component.launch.xml)
  - 相关文档：https://deepwiki.com/autowarefoundation/autoware_universe#2

- **预期洞察**：
  1. 理解 `PlannerManager` 如何管理和调度多个场景模块
  2. 掌握行为状态机的状态定义与转换逻辑
  3. 学习速度规划插件（CrosswalkModule、TrafficLightModule）的实现

- **验证方式**：
  ```bash
  colcon test --packages-select autoware_behavior_path_planner
  ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once
  ros2 topic echo /planning/scenario_planning/trajectory --once
  ```

---

## 9. 控制系统 - MPC/PID 与安全门控

- **为什么重要**：控制系统将轨迹转化为实际的转向、加速、制动指令。

- **源码入口**：
  - MPC 横向控制：[control/mpc_lateral_controller](https://github.com/autowarefoundation/autoware.universe/tree/main/control/mpc_lateral_controller)
  - PID 纵向控制：[control/pid_longitudinal_controller](https://github.com/autowarefoundation/autoware.universe/tree/main/control/pid_longitudinal_controller)
  - Pure Pursuit（core）：[autoware_simple_pure_pursuit](https://github.com/autowarefoundation/autoware_core/tree/main/control/autoware_simple_pure_pursuit)
  - 自动紧急制动：[control/autonomous_emergency_braking](https://github.com/autowarefoundation/autoware.universe/tree/main/control/autonomous_emergency_braking)
  - 控制验证器：[control/control_validator](https://github.com/autowarefoundation/autoware.universe/tree/main/control/control_validator)
  - 命令门控：[control/vehicle_cmd_gate](https://github.com/autowarefoundation/autoware.universe/tree/main/control/vehicle_cmd_gate)
  - 启动文件：[tier4_control_component.launch.xml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/components/tier4_control_component.launch.xml)
  - 相关文档：https://deepwiki.com/autowarefoundation/autoware_universe#4

- **预期洞察**：
  1. 理解 MPC 的状态空间模型与代价函数设计
  2. 掌握多层安全校验机制（控制验证器 → 命令门控）
  3. 学习 AEB 的触发条件与制动策略

- **验证方式**：
  ```bash
  colcon test --packages-select autoware_mpc_lateral_controller autoware_autonomous_emergency_braking
  ros2 topic echo /control/command/control_cmd --once
  ```

---

## 10. 可观测性与质量保障

- **为什么重要**：日志、指标、追踪是理解系统运行时行为和调试问题的关键。

- **源码入口**：
  - 系统监控：[system/system_monitor](https://github.com/autowarefoundation/autoware.universe/tree/main/system/system_monitor)
  - 组件状态监控：[system/component_state_monitor](https://github.com/autowarefoundation/autoware.universe/tree/main/system/component_state_monitor)
  - 诊断聚合：[system/diagnostic_graph_aggregator](https://github.com/autowarefoundation/autoware.universe/tree/main/system/diagnostic_graph_aggregator)
  - 测试框架（core）：[autoware_test_utils](https://github.com/autowarefoundation/autoware_core/tree/main/common/autoware_test_utils)
  - 规划测试管理器（core）：[autoware_planning_test_manager](https://github.com/autowarefoundation/autoware_core/tree/main/common/autoware_planning_test_manager)
  - CI 配置：[.github/workflows/](https://github.com/autowarefoundation/autoware.universe/tree/main/.github/workflows)

- **预期洞察**：
  1. 理解 `rclcpp` 日志系统与诊断消息发布机制
  2. 掌握 gtest/gmock 单元测试与 launch_test 集成测试的使用
  3. 学习 CI/CD 自动化测试流程

- **验证方式**：
  ```bash
  # 运行全部测试
  colcon test

  # 查看诊断信息
  ros2 topic echo /diagnostics

  # 使用 ros2_tracing 进行性能追踪
  ros2 trace start
  ```

---

## 11. 工程效率与发布

- **为什么重要**：理解构建、缓存、镜像发布策略，确保可重复构建与快速迭代。

- **源码入口**：
  - Docker 构建配置：[docker-bake.hcl](https://github.com/autowarefoundation/autoware/blob/main/docker/docker-bake.hcl)
  - 镜像标签策略：[docker-build-and-push action](https://github.com/autowarefoundation/autoware/tree/main/.github/actions/docker-build-and-push)
  - 依赖更新工作流：[create-prs-to-update-vcs-repositories.yaml](https://github.com/autowarefoundation/autoware/blob/main/.github/workflows/create-prs-to-update-vcs-repositories.yaml)
  - ccache 配置：构建脚本中的 `CCACHE_*` 环境变量

- **预期洞察**：
  1. 理解 `docker buildx bake` 的多架构构建策略
  2. 掌握镜像标签规范（日期标签、release 标签）
  3. 学习依赖版本自动更新的 PR 工作流

- **验证方式**：
  ```bash
  # 本地构建验证
  cd autoware
  docker buildx bake --file docker/docker-bake.hcl universe-devel
  ```

---

【扩展维度】

## 性能与实时性

- **热点路径**：TensorRT 推理（感知）、NDT 配准（定位）、MPC QP 求解（控制）
- **时延预算**：端到端 < 100ms（10Hz 决策循环）
- **并发模型**：ROS 2 MultiThreadedExecutor + intra-process 零拷贝
- **入口**：`autoware_tensorrt_common` 的 `TrtCommon::enqueueV3`
- **验证**：使用 `ros2_tracing` 记录关键 span，绘制时延直方图

---

## 测试与质量

- **测试金字塔**：gtest 单元测试 + launch_test 集成测试 + 仿真场景测试
- **覆盖率**：lcov + CodeCov，多架构支持
- **契约测试**：对跨模块消息运行 `ros2 interface show` 核对字段
- **入口**：各模块 `test/` 目录，CI 配置 `.github/workflows/`
- **验证**：`colcon test --packages-select <package> --event-handlers console_cohesion+`

---

## 安全与功能安全

- **多层防护**：控制验证器 → 命令门控 → AEB
- **降级策略**：MRM（最小风险操作）实现
- **容器安全**：最小权限原则、ROS 2 通信加固
- **入口**：`autoware_universe/control/autoware_vehicle_cmd_gate/`，`autoware_universe/system/autoware_mrm_handler/`
- **验证**：模拟异常输入场景，观察系统降级行为

---

## 生态与扩展

- **插件机制**：行为规划场景模块、速度规划插件
- **仿真器集成**：AWSIM、CARLA、LGSVL
- **传感器适配**：Nebula LiDAR 驱动、transport_drivers
- **入口**：`autoware_universe/simulator/`，`sensor_component/`
- **验证**：尝试添加新的场景模块或替换控制器算法

---

## 部署与运维

- **容器化**：Docker 镜像层次（devel → runtime → cuda）
- **配置管理**：ROS 2 参数系统 + YAML 预设
- **参数标定**：传感器内参/外参、车辆动力学参数
- **入口**：`autoware/docker/Dockerfile`，`autoware_launch/autoware_launch/config/`
- **验证**：
  ```bash
  ros2 launch autoware_launch planning_simulator.launch.xml \
      map_path:=$HOME/autoware_map/sample-map-planning \
      vehicle_model:=sample_vehicle \
      sensor_model:=sample_sensor_kit
  ```

---

【关键依据（来源与摘录）】

| 来源 | 摘要 |
|------|------|
| https://deepwiki.com/autowarefoundation/autoware#1 | 元仓库管理 `.repos` 文件、Docker 镜像层次、CI/CD 工作流 |
| https://deepwiki.com/autowarefoundation/autoware_core#1 | 5 层架构、稳定接口、插件架构设计模式 |
| https://deepwiki.com/autowarefoundation/autoware_universe#2 | 规划系统分层：任务→场景→行为→运动 |
| https://deepwiki.com/autowarefoundation/autoware_universe#3 | 感知流水线：检测→跟踪→预测 |
| https://deepwiki.com/autowarefoundation/autoware_universe#4 | 控制系统：MPC/PID、安全门控、AEB |
| https://deepwiki.com/autowarefoundation/autoware_msgs#1 | 消息包职责划分与数据流 |
| https://deepwiki.com/autowarefoundation/autoware_launch#2 | 4 种运行模式与参数传递链 |
| https://deepwiki.com/autowarefoundation/autoware_lanelet2_extension#4 | 监管元素、查询 API、可视化 API |

---

【学习顺序（建议）】

**第一阶段：架构与基础（1-2 天）**
1. 架构与依赖编排（autoware 元仓库）→ 理解构建链路
2. 消息与接口契约（autoware_msgs）→ 理解数据流边界
3. 启动编排与运行模式（autoware_launch）→ 掌握验证入口

**第二阶段：核心抽象（2-3 天）**
4. 核心抽象层（autoware_core）→ 理解稳定接口与设计模式
5. 地图系统（autoware_lanelet2_extension）→ 理解语义地图

**第三阶段：功能链路（1-2 周）**
6. 定位系统 → 理解车辆如何确定自身位置
7. 感知系统 → 理解环境感知流程
8. 规划系统 → 理解决策层如何生成轨迹
9. 控制系统 → 理解轨迹如何执行

**第四阶段：工程与运维（3-5 天）**
10. 可观测性与质量保障 → 理解测试与监控体系
11. 工程效率与发布 → 理解 CI/CD 与部署

---

【回归验证与回退方案】

**双验证场景（必须跑通）**

| 场景 | 启动命令 | 验证点 |
|------|----------|--------|
| 规划仿真 | `ros2 launch autoware_launch planning_simulator.launch.xml ...` | 可视化路径规划、设置目标点、车辆自动行驶 |
| 日志回放 | `ros2 launch autoware_launch logging_simulator.launch.xml` + `ros2 bag play <bag>` | 定位/感知/规划话题正常输出 |

**验证清单**

- [ ] 成功构建 Autoware 全栈（`colcon build`）或使用预编译 Docker 镜像
- [ ] 规划仿真模式下车辆可自主行驶到目标点
- [ ] 日志回放模式下各模块话题正常发布
- [ ] 核心模块单元测试通过
- [ ] 消息契约快照与基线一致

**回退策略**

| 阻塞场景 | 回退方案 |
|----------|----------|
| 编译失败 | 使用预编译 Docker 镜像 `ghcr.io/autowarefoundation/autoware:universe-cuda` |
| 某模块理解阻塞 | 先跳过实现细节，关注接口与数据流，后续深入 |
| 版本不兼容 | 回退到 `autoware.repos` 中指定的稳定版本标签 |
| 参数/标定缺失 | 使用 `sample_vehicle` 和 `sample_sensor_kit` 默认配置 |

---

【缺口与后续动作】

**已知缺口**

| 缺口 | 原因 | 影响 |
|------|------|------|
| 具体版本号未锁定 | 使用 main 分支 | API 可能有变化 |
| 深度学习模型训练流程 | 项目主要提供推理代码 | 需参考独立训练仓库 |
| 实车部署细节 | 需硬件环境验证 | 需准备传感器与车辆接口 |
| 特定仿真器集成 | CARLA/AWSIM 配置复杂 | 需额外配置文档 |
| 实时性调优 | 需实际负载测试 | 需 PREEMPT_RT 内核 |

**后续动作**

1. 克隆仓库后使用 `git log` 确认具体 commit，必要时切换到特定 release tag
2. 生成"功能链路 × 仓库/包/话题"索引表，映射到 11 条主线子系统
3. 补齐"最小参数集"与"消息契约快照"样例，纳入回归
4. 针对实车部署，准备传感器硬件与车辆接口适配文档
5. 针对性能调优，记录关键节点周期与队列延时

---

【功能链路 × 仓库/包/话题 映射表】

```
感知链路:
  传感器输入 → [autoware_sensing_msgs]
    → 地面分割 [autoware_universe/perception/autoware_ground_segmentation]
    → 物体检测 [autoware_universe/perception/autoware_lidar_centerpoint]
      话题: /perception/object_recognition/detection/objects
    → 物体跟踪 [autoware_universe/perception/autoware_multi_object_tracker]
      话题: /perception/object_recognition/tracking/objects
    → 轨迹预测 [autoware_universe/perception/autoware_map_based_prediction]
      话题: /perception/object_recognition/prediction/objects

规划链路:
  路线输入 → [autoware_planning_msgs/LaneletRoute]
    → 任务规划 [autoware_core/planning/autoware_mission_planner]
    → 行为路径规划 [autoware_universe/planning/autoware_behavior_path_planner]
      话题: /planning/scenario_planning/lane_driving/behavior_planning/path
    → 行为速度规划 [autoware_universe/planning/autoware_behavior_velocity_planner]
    → 运动规划 [autoware_core/planning/autoware_motion_velocity_planner]
      话题: /planning/scenario_planning/trajectory

控制链路:
  轨迹输入 → [autoware_planning_msgs/Trajectory]
    → MPC 横向控制 [autoware_universe/control/autoware_mpc_lateral_controller]
    → PID 纵向控制 [autoware_universe/control/autoware_pid_longitudinal_controller]
    → 控制验证器 [autoware_universe/control/autoware_control_validator]
    → 命令门控 [autoware_universe/control/autoware_vehicle_cmd_gate]
      话题: /control/command/control_cmd
    → 车辆接口 [autoware_vehicle_msgs]
```

---

*本源码阅读目标由 AI 自动生成，基于 Autoware 生态系统 6 大核心仓库的官方文档与 DeepWiki 知识库整理而成。*

*生成工具: Claude Code + Codex MCP + DeepWiki MCP*
*生成时间: 2026-01-03*
