- [CAVE\_EXPLORATION](#cave_exploration)
  - [编译](#编译)
  - [运行](#运行)
  - [注意事项:](#注意事项)

# CAVE_EXPLORATION
溶洞探索的项目代码，在MARSIM-FUEL-FAEL上面做的改进。

## 编译
本项目直接用docker编译，无需安装任何依赖，请按照`docker`目录中的`README.md`进行操作。

## 运行
**直接运行`shell_commands`目录下的.sh脚本文件**


1. 溶洞仿真(Cave-Exploration目录下)
```bash
cd shell_commands
./run_cave_sim.sh
```
2. 竹林仿真
```bash
cd shell_commands
./run_bamboo_sim.sh
```

3. 现实小环境(Cave-Exploration目录下)
```bash
cd shell_commands
./run_small_real.sh
```

1. 现实大环境(Cave-Exploration目录下)
```bash
cd shell_commands
./run_large_real.sh
```

## 注意事项:
1. 现实环境中要用到FAST_LIO, 因此请先配置好相关依赖和编译配置好FAST_LIO

2. 开启探索的时候把FAST_LIO的pcd保存功能关掉，在`FAST_LIO/config/avia.yaml`.

3. 该场序有两部分的参数文件，分别在`fuel_planner/active_perception/resources`和`fuel_planner/exploration_manager/launch`，可按照需要进行相应调整.

4. `fuel_planner/active_perception/resources`里的参数文件设计的参数含义可见`config_cave.yaml`

5. `exploration_data_files`存着每个预处理的时间消耗，以及planning的时间消耗. 
