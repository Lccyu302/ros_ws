# 北极熊哨兵 ROS 2 仿真导航路径规划学习总结

本文围绕北极熊哨兵导航仿真项目的代码学习和参数调试过程进行总结。项目基于 ROS 2 与 Nav2，主要涉及仿真环境、点云与惯性里程计、坐标变换、地形分析、代价地图、全局路径规划和全向底盘路径跟踪。当前总结结合工作空间 `/home/l/ros_ws` 中的源码和配置，重点记录从理解系统到定位调参问题的学习过程。

## 一、项目学习路线

### 1. 从启动流程理解系统结构

项目启动时会根据 `world` 选择仿真世界、栅格地图和先验点云，并通过 namespace 管理机器人节点和 topic。导航系统的总启动逻辑位于 [bringup_launch.py](src/pb2025_sentry_nav/pb2025_nav_bringup/launch/bringup_launch.py)：当 `slam:=True` 时进入建图流程，当 `slam:=False` 时加载已有地图并进行定位，两种模式都会继续启动 Nav2 导航节点。

典型的已有地图导航命令为：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \\
  world:=rmuc_2025 \\
  slam:=False
```

建图命令为：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \\
  world:=rmuc_2025 \\
  slam:=True
```

首先明确了建图、定位和导航的区别：SLAM 负责在线建立二维地图；已有地图模式通过 `map_server` 加载栅格地图，并利用 Point-LIO 和 small GICP 进行定位；规划和控制则由 Nav2 的 planner、controller、行为树和速度平滑节点共同完成。

### 2. 按数据流学习各功能包

项目的主要数据链路可以概括为：

```text
仿真点云/IMU
  → ign_sim_pointcloud_tool
  → Point-LIO
  → loam_interface、sensor_scan_generation
  → terrain_analysis / terrain_analysis_ext
  → local_costmap / global_costmap
  → Theta* 全局规划
  → OmniPidPursuitController
  → velocity_smoother / fake_vel_transform
  → 仿真底盘
```

仿真器输出的点云首先经过 [point_cloud_converter.cpp](src/pb2025_sentry_nav/ign_sim_pointcloud_tool/src/point_cloud_converter.cpp) 适配。该节点根据点云几何信息补充 `ring` 和 `time` 字段，并将输入 topic `livox/lidar` 转换为 Point-LIO 使用的 `velodyne_points`。这说明仿真传感器输出不一定能够直接作为算法输入，算法对点云字段和扫描线组织方式也有要求。

Point-LIO 的状态估计和配准点云通过 [loam_interface.cpp](src/pb2025_sentry_nav/loam_interface/src/loam_interface.cpp) 与 [sensor_scan_generation.cpp](src/pb2025_sentry_nav/sensor_scan_generation/src/sensor_scan_generation.cpp) 进入导航链路。`loam_interface` 主要处理 `lidar_odom` 与 `odom` 之间的坐标关系，`sensor_scan_generation` 对点云和里程计进行时间同步，并发布导航需要的 odometry、TF 和传感器扫描数据。因此，Point-LIO 不只是输出一个位置数值，还必须和项目的 TF 树、时间戳以及机器人基座坐标保持一致。

### 3.TF、地图和定位

项目中需要重点理解的坐标系包括：

```text
map → odom → base_footprint / gimbal_yaw → front_mid360
```

其中，`map` 用于全局地图和全局路径，`odom` 表示连续的局部里程计坐标，`gimbal_yaw` 是导航和运动使用的机器人基座相关坐标，`front_mid360` 是雷达坐标系。先验 PCD 用于点云定位或重定位，二维栅格地图则主要服务于 Nav2 的全局代价地图和全局规划，两者不能混为一谈。

TF 是整个导航系统的连接基础。排查导航问题时，需要确认 `map→odom`、`odom→gimbal_yaw` 和雷达到机器人基座的变换是否存在、是否连续、时间戳是否使用仿真时钟。常用检查命令如下：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom gimbal_yaw
ros2 run tf2_tools view_frames
```

仿真时所有相关节点应统一使用 `use_sim_time:=True`，否则可能出现 TF extrapolation、消息过期或点云与里程计同步失败等问题。

### 4. Nav2 的规划与控制分工

项目的具体参数位于 [simulation/nav2_params.yaml](src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml)。当前仿真配置中的全局规划器为：

```yaml
GridBased:
  plugin: "nav2_theta_star_planner/ThetaStarPlanner"
```

Theta* 根据全局代价地图搜索起点到目标点的全局路径。它不直接控制底盘，也不负责解决所有近距离动态障碍。

局部路径跟踪器为：

```yaml
FollowPath:
  plugin: "pb_omni_pid_pursuit_controller::OmniPidPursuitController"
```

[omni_pid_pursuit_controller.cpp](src/pb2025_sentry_nav/pb_omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp) 会将全局路径转换到机器人坐标系，剪除已经经过的路径点，根据速度计算 lookahead 点，再通过 PID、曲率限速、接近目标减速和碰撞检查生成全向 `vx`、`vy`、`wz`。因此，“全局路径正确”并不代表机器人一定能够通过，local costmap、控制器碰撞检查、速度限制和底盘执行仍会影响最终结果。

---

## 二、导航调参总结

### 1. 膨胀半径与全局、局部代价地图

`inflation_radius` 表示障碍物代价向外扩散的范围。障碍物周围会形成从高代价到低代价的区域，使规划器倾向于远离障碍物。它不是机器人半径，也不是机器人绝对安全距离，最终安全性还与 `robot_radius`、真实 footprint、地图分辨率、定位误差、控制跟踪误差和运动速度有关。

当前仿真配置中，local 和 global 的膨胀半径并不相同：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: 0.15

global_costmap:
  global_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: 0.7
```

全局代价地图的主要任务是为 Theta* 提供全局安全路线，因此较大的膨胀半径会使路线更保守；局部代价地图主要服务机器人附近的实时避障和控制器碰撞检查，过大的膨胀半径可能堵死窄洞或窄通道。调试过程中曾尝试将 `0.7` 调整为 `0.3` 或 `0.45`，由此得到的经验是：全局和局部可以分开调，不能只根据某一张地图的显示结果下结论。

膨胀半径调大，通常能够增加安全距离，但可能使窄洞不可行；调小则能够提高通过性，但路径更容易贴边，对定位误差、控制误差和转弯空间更加敏感。上坡蛇形拐弯处尤其不能只为了过洞而无限减小膨胀半径，还需要同时检查机器人 footprint、局部路径、曲率减速和实际车体外轮廓。比较合理的思路是保持全局层相对保守，再根据洞口实际宽度适度调整局部层，并通过低速和曲率限速降低撞角风险。

### 2. 过洞问题的定位方法

仿真中出现过远处路径正常、机器人接近洞口后无法通过的情况。该现象不能直接归结为 Theta* 或膨胀半径，需要先区分全局规划、局部代价地图和控制器碰撞检查。

本项目的输入关系是：

```text
terrain_analysis      → terrain_map      → local_costmap
terrain_analysis_ext  → terrain_map_ext  → global_costmap
```

`terrain_analysis` 主要处理机器人附近约 4 m 的地形，`terrain_analysis_ext` 负责更远范围。因此，洞口位于机器人前方近距离时，通常应优先检查 `terrain_map` 和 local costmap，而不是只修改 global costmap。如果只修改了 global costmap 的参数，local costmap 仍可能把洞口封死，表面上就会出现“改了参数但没有效果”。

过洞问题的排查顺序：

1. 观察 global plan 是否本来就没有穿过洞口。若没有，检查静态地图、global costmap、global inflation、机器人半径和 Theta*。
2. 如果 global plan 穿过洞口，观察 local costmap 是否将洞口封死。若封死，检查 `terrain_map`、local 层参数以及高度、强度和距离筛选。
3. 如果 local costmap 没有完全封死，检查控制器的局部路径采样和碰撞检查。控制器会判断路径采样点是否达到 `INSCRIBED_INFLATED_OBSTACLE`，视觉上存在缝隙并不代表控制器允许通过。
4. 如果控制器能够输出速度，再检查 `cmd_vel` 的 remap、`fake_vel_transform` 和底盘电控是否真正执行。

在 RViz 中应同时观察 `terrain_map`、`terrain_map_ext`、local/global costmap、global plan、local plan、机器人 footprint 和 lookahead point，确认究竟是点云误判、voxel 层标记、inflation 扩散、footprint 过大，还是控制器碰撞检查导致无法通过。

### 3. 地形分析与 `max_obstacle_height`

`IntensityVoxelLayer` 对输入点云会依次进行高度、强度和距离筛选，满足条件后才会写入 voxel grid 和 costmap。其实现位于 [intensity_voxel_layer.cpp](src/pb2025_sentry_nav/pb_nav2_plugins/src/layers/intensity_voxel_layer.cpp)。

调参时容易混淆两类 `max_obstacle_height`：

- `intensity_voxel_layer` 下的参数属于图层级高度限制。
- `terrain_map` 或 `terrain_map_ext` 下的参数属于具体 observation source 的高度限制。

可以将其理解为两道筛选：

```text
点云输入
  → 数据源级高度筛选
  → 图层级高度筛选
  → intensity 筛选
  → 距离筛选
  → voxel 标记
  → costmap 障碍
```

近处洞口应优先查看 `local_costmap` 下 `terrain_map` 的相关参数；远处障碍则查看 `global_costmap` 下 `terrain_map_ext` 的相关参数。此前将 global costmap 的 `max_obstacle_height` 从 `2.0` 改为 `0.6`，如果洞口实际由 local costmap 的 `terrain_map` 处理，就不会解决近处封路问题。

但降低高度上限并不是通用解决方案。若洞壁、横梁或其他结构是真实障碍，过度降低高度可能使点云被过滤后产生碰撞风险。因此应先确认被过滤的点是什么，再决定修改高度范围。

`terrain_analysis` 中的 `useSorting`、`quantileZ` 和 `clearDyObs` 也会影响洞口和坡面的结果。当前配置为：

```yaml
useSorting: True
quantileZ: 0.2
clearDyObs: True
```

`useSorting` 使用分位数思路估计地面，对坡面通常更友好，但在洞口、台阶或稀疏点云区域可能改变地面判断；`quantileZ` 影响地面参考高度；`clearDyObs` 用于动态障碍处理，可能影响点云在时序上的稳定性。因此过洞不稳定时，不能只调整 inflation，还应对比 `terrain_map` 的实际输出，并一次只改变一个地形分析参数。

### 4. 速度参数与电控的关系

导航配置中的速度上限只是导航链允许发布的指令范围，不等于底盘电控一定能够稳定执行。例如当前参数中：

```yaml
FollowPath:
  v_linear_max: 2.5
  v_angular_max: 3.0

velocity_smoother:
  max_velocity: [2.5, 2.5, 3.0]
```

最终运动能力还受电机、电控限流、电池电压、地面摩擦、坡度、负载和机械结构影响。如果导航指令超过电控物理上限，可能出现速度饱和、里程计跟不上、路径跟踪误差增大、转弯来不及减速甚至运动不稳定等现象。

实车调参不能直接照搬仿真速度。应先与电控确认实际最大线速度、横向速度、角速度、加速度和减速度，再从较低速度开始，建议先使用仿真速度的约 50% 作为保守起点，并确保遥控接管和急停有效。待直线、转弯、过洞和上坡均能稳定完成后，再逐步提高速度。

### 5. 调参目标和顺序

```text
先不撞 → 能通过目标通道 → 路径稳定 → 提高速度 → 缩短到达时间
```

先确认 `/clock`、`velodyne_points`、`odometry`、TF、`terrain_map` 和 costmap 数据正常，再确认实际加载的 planner/controller 插件和参数。调参阶段可以暂时使用独立节点模式：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \\
  world:=rmuc_2025 \\
  slam:=False \\
  use_composition:=False
```

之后按照“地形与障碍识别 → global/local inflation → footprint → 控制器 → 速度”的顺序进行。控制器方面优先关注 `min_lookahead_dist`、`max_lookahead_dist`、`lookahead_time`、最大线速度、曲率阈值和高曲率减速比例，最后再细调 PID。每次只修改一个变量，并记录修改前后的路径、costmap、是否过洞、最近障碍距离、是否撞角和速度表现。


---
