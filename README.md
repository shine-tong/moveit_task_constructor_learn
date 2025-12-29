# MoveIt Task Constructor (MTC) Demo

## 1. 环境要求

- ROS Noetic
- MoveIt
- MoveIt Task Constructor
- Python 3.8

## 2. 源码安装 `moveit_task_constructor`

```bash
# 进入 catkin 工作空间
cd ~/mtc_ws/src

# 克隆项目（如果尚未克隆）
git clone --recursive https://github.com/shine-tong/moveit_task_constructor_learn.git

# 安装依赖
cd ..
rosdep install --from-paths src --ignore-src -r -y

# 编译
catkin build
source devel/setup.bash
```

## 3. MTC 示例脚本

项目包含三个 MoveIt Task Constructor 示例（**运行示例前先** `source` **工作空间**）：

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
│   ├── moveit_task_constructor/  # MTC 子模块 (submodule)
│   ├── mtc_demo/                 # MTC 示例包
│   │   └── scripts/              # Python 脚本
│   │       ├── cartesian.py
│   │       ├── constrained.py
│   │       ├── convert_msgs.py
│   │       ├── modular.py
│   │       ├── multi_planner.py
│   │       └── pick_place.py
│   └── sa2000h_moveit_config_0521/
│       ├── config/               # 配置文件
│       └── launch/               # 启动文件
├── build/
└── devel/
```

## 5. VSCode 配置

### 5.1 解决 `moveit.task_constructor` 无法解析导入

在工作空间根目录创建 `.vscode/settings.json`：

```json
{
    "python.analysis.extraPaths": [
        "${workspaceFolder}/devel/lib/python3/dist-packages",
        "${workspaceFolder}/src/moveit_task_constructor/core/python/src"
    ]
}
```

配置后按 `Ctrl+Shift+P` 输入 "Reload Window" 重新加载窗口。

### 5.2 生成 `.pyi` 类型存根文件（自动补全）

pybind11 编译的 `.so` 模块需要生成类型存根才能获得自动补全：

```bash
# 安装 pybind11-stubgen
pip3 install pybind11-stubgen

# source 工作空间环境
source ~/mtc_ws/devel/setup.bash

# 生成存根文件
pybind11-stubgen pymoveit_mtc -o ~/mtc_ws/devel/lib/python3/dist-packages/
```

> 注意：生成过程中可能会有一些 ERROR 警告（C++ 类型无法完全解析），但不影响基本的自动补全功能。