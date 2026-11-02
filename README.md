#### 导航2.2.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
##rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> [!NOTE]
> 推荐使用 --symlink-install 选项来构建你的工作空间，因为 pb2025_sentry_nav 广泛使用了 launch.py 文件和 YAML 文件。这个构建参数会为那些非编译的源文件使用符号链接，这意味着当你调整参数文件时，不需要反复重建，只需要重新启动即可。

### 2.3 Running

可使用以下命令启动，在 RViz 中使用 `Nav2 Goal` 插件发布目标点。

#### 2.3.1 仿真

单机器人：

导航模式：

```bash
source ~/ros_ws/install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
world:=rmuc_2025 \
slam:=False
```
ros2 launch pb2025_robot_description robot_description_launch.py \
  namespace:=red_standard_robot1 \
  robot_name:=pb2025_infantry_robot \
  use_sim_time:=True \
  use_rviz:=False

建图模式：

```bash
source ~/ros_ws/install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
slam:=True
```

保存栅格地图：`ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>  --ros-args -r __ns:=/red_standard_robot1`

多机器人 (实验性功能) :

当前指定的初始位姿实际上是无效的。TODO: 加入 `map` -> `odom` 的变换和初始化

```bash
ros2 launch pb2025_nav_bringup rm_multi_navigation_simulation_launch.py \
world:=rmul_2024 \
robots:=" \
red_standard_robot1={x: 0.0, y: 0.0, yaw: 0.0}; \
blue_standard_robot1={x: 5.6, y: 1.4, yaw: 3.14}; \
"
```

#### 2.3.2 实车

建图模式：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
slam:=True \
use_robot_state_pub:=True
```

保存栅格地图：`ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>  --ros-args -r __ns:=/red_standard_robot1`

导航模式：

注意修改 `world` 参数为实际地图的名称

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=<YOUR_WORLD_NAME> \
slam:=False \
use_robot_state_pub:=True
```

#### 仿真2.2.3 Build

```sh
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```sh
source ~/ros_ws/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
>加上参数
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_UPDATES_DISCONNECTED=ON

```

### 2.3 Running

启动仿真环境

```sh
source ~/ros_ws/install/setup.bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```
 [!NOTE]
> **注意：需要点击 Gazebo 左下角橙红色的 `启动` 按钮**

#### 2.3.1 Test Commands

控制机器人移动

```sh
source ~/ros_ws/install/setup.bash
ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=0.3 -p w:=0.3
#根据提示进行输入，支持平移与自旋
```

机器人云台

```sh
ros2 run rmoss_gz_base test_gimbal_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
#根据提示进行输入，支持绝对角度控制
```

机器人射击

```sh
ros2 run rmoss_gz_base test_shoot_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
#根据提示进行输入
```


