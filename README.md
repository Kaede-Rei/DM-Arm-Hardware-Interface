# Arm Frame Test

## install Pinocchio & TSID
```bash
sudo apt update
# 安装 Pinocchio 和 碰撞检测库 FCL
sudo apt install ros-humble-pinocchio \
    ros-humble-hpp-fcl \
    libboost-python-dev \
    libeigen3-dev \
    ros-humble-eigenpy 
# 安装可视化工具
pip install meshcat
# 克隆 TSID 源码
cd src
git clone --recursive https://github.com/stack-of-tasks/tsid.git
cd ..
rosdep install --from-paths src -iry
# 扩大 Swap 空间以避免编译过程中内存不足导致的卡崩
sudo swapoff /swapfile
sudo dd if=/dev/zero of=/swapfile bs=1M count=16384 status=progress
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
free -h
# 编译工作空间，启用 Python 接口和 FCL 支持，使用串行构建器以避免卡崩
colcon build --executor sequential \
    --symlink-install --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_PYTHON_INTERFACE=ON \
    -DPINOCCHIO_USE_HPP_FCL=ON
# 编译完记得添加 COLCON_IGNORE 文件
```

## install Tesseract
```bash
# 安装系统构建工具和依赖
sudo apt update
sudo apt install python3-vcstool python3-colcon-common-extensions git
# 克隆 Tesseract 仓库
cd src
git clone https://github.com/tesseract-robotics/tesseract_ros2.git -b humble
vcs import < tesseract_ros2/dependencies.repos
# 安装 Tesseract 依赖
cd ..
rosdep install --from-paths src -iry
# 编译工作空间，使用串行构建器以避免卡崩
colcon build --executor sequential \
    --symlink-install --cmake-args \
    -DTESSERACT_BUILD_FCL=OFF \
    -DBUILD_RENDERING=OFF \
    -DBUILD_STUDIO=OFF
```

## run
