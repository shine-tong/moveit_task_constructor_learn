# MoveIt Task Constructor (MTC) Demo

## 1. 环境要求

- ROS Noetic
- MoveIt
- MoveIt Task Constructor
- Python 3.8

## 2. 安装

```bash
# 进入 catkin 工作空间
cd ~/catkin_ws/src

# 克隆项目（如果尚未克隆）
git clone https://github.com/shine-tong/moveit_task_constructor_learn.git

# 安装依赖
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# 编译
catkin_make
source devel/setup.bash
```

## 3. MTC 示例脚本

项目包含三个 MoveIt Task Constructor 示例：

### 3.1 笛卡尔路径规划 (`cartesian.py`)

演示笛卡尔空间运动：
- X/Y 方向平移
- 绕 Z 轴旋转
- 关节空间偏移
- 移动到预设姿态

```bash
# 先启动 MTC demo 环境
roslaunch sa2000h_moveit_config_0521 demo_mtc.launch

# 然后运行脚本
python3 src/scripts/cartesian.py
```

### 3.2 抓取放置 (`pick_place.py`)

完整的 Pick & Place 任务流程：
- 场景物体创建
- 抓取姿态生成
- 物体抓取与举起
- 物体放置与释放

```bash
# 先启动 MTC demo 环境
roslaunch moveit_task_constructor_demo demo.launch

# 然后运行抓取放置脚本
python3 src/scripts/pick_place.py
```

### 3.3 模块化任务 (`modular.py`)

演示如何使用 SerialContainer 组织模块化子任务：

```bash
# 先启动 MTC demo 环境
roslaunch sa2000h_moveit_config_0521 demo_mtc.launch

# 然后运行脚本
python3 src/scripts/modular.py
```

## 4. 目录结构

```
├── src/
│   ├── sa2000h_moveit_config_0521/
│   │   ├── config/          # 配置文件
│   │   │   ├── kinematics.yaml
│   │   │   ├── joint_limits.yaml
│   │   │   ├── ompl_planning.yaml
│   │   │   └── ...
│   │   └── launch/          # 启动文件
│   │       ├── demo.launch
│   │       ├── demo_mtc.launch
│   │       └── ...
│   └── scripts/             # Python 脚本
│       ├── cartesian.py
│       ├── pick_place.py
│       ├── modular.py
│       └── convert_msgs.py
├── build/
└── devel/
```