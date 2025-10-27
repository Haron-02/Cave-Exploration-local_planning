- [本Docker使用方式](#本docker使用方式)
  - [一些准备工作](#一些准备工作)
  - [首次编译程序](#首次编译程序)
  - [运行(只能运行仿真的)](#运行只能运行仿真的)
  - [再次编译程序](#再次编译程序)
  - [重新生成新的容器](#重新生成新的容器)

# 本Docker使用方式
**注意: 一切操作都是在docker目录下进行的！若直接使用本md的指令，请先cd到docker目录！**

**注意: 一切操作都是在docker目录下进行的！若直接使用本md的指令，请先cd到docker目录！**

**注意: 一切操作都是在docker目录下进行的！若直接使用本md的指令，请先cd到docker目录！**

## 一些准备工作
**如果你执行docker命令需要带sudo，请先执行以下操作**
1. 创建 docker 分组(如果这个分组已经存在会报错)
```bash
sudo groupadd docker
```
2. 将当前用户加入docker分组
```bash
sudo gpasswd -a ${USER} docker
```
3. 切换当前会话到新 group 或者重启，此操作仅会对当前终端有效。要想一劳永逸，最好将电脑注销后再次启动
```bash
newgrp - docker
```
4. 重新进入docker目录

## 首次编译程序
1. 首先进入到docker目录，执行下面指令创建docker镜像。
```bash
make build 
```

2. 接着执行下面`build.sh`构建容器并把宿主机当前的ROS的代码通过容器数据卷的方式共享，然后使用容器环境编译代码。`build.sh`编译完后会执行`back_to_host.sh`将编译得到的devel拷贝回宿主机当前ROS工作空间，并修改权限。
```bash
./build.sh
```

## 运行(只能运行仿真的)
除非你需要重新编译，否则不需要再执行上面的编译程序的步骤，直接在宿主机工作空间运行程序即可，无需再用docker。

**运行程序的指令已经写入`run.sh`。**
```bash
./run.sh
```
**noted:** 如果QT报错，可以新开一个终端，进入docker目录后再次执行`./run.sh`.
## 再次编译程序
**为了不让程序每次都从头编译，在`build.sh`中容器退出后默认是不会删除容器的。**
因此你可以直接进入之前的容器继续在之前的基础上编译。

**当然，一切的操作都在`compile.sh`写好了。**`compile.sh`同样会调用`back_to_host.sh`将编译结果拷贝回来并作相应处理。
```bash
./compile.sh
```

## 重新生成新的容器
**请先执行`sudo docker rm -f cave_exp `把旧的容器删掉。其中cave_exp是`./build`生成的容器的默认名字。**




