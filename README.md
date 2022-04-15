# Cugo Ros2

## はじめに
こちらはCugoV3 Odometry　PluginなしのROS2Packageになります。

## 必要なもの
CugoV3

Jetson Nano(ubuntu 20.04)

Lidar(今回はRPLIDAR A2M8を利用)

## 環境
Jetson Nano Ubuntu 20.04 
ROS2 Galactic

## 手順

1. JETSON に20.04のイメージを導入

https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image

2. Jetson のPWMをONにします

https://qiita.com/kitazaki/items/a445994f1f46a1b15f78


3. ros2 galacticをインストールします。

https://qiita.com/porizou1/items/30452faa5f108cfd8995

4. cugo repositoryをビルドします。

~~~
mkdir -p ~/cugo_ws/src
cd ~/cugo_ws/src
git clone -b galactic-devel https://github.com/M2labo/cugo_ros2.git
cd ..
sudo rosdep init
rosdep update 
rosdep install --from-paths src/ros2cugo -i
colcon build --symlink-install
~~~

5. cugo の利用

~~~
source instaill/setup.bash
ros2 launch cugo_bringup robot.launch.py
~~~

6. navigation2の利用

    1. rplidar_ros(RPLIDARのROS2パッケージ)をインストール
    
    ~~~
    git clone https://github.com/youtalk/rplidar_ros/urg_node.git
    rosdep install --from-paths src -i
    colcon build --symlink-install
    ~~~

    2. navigation2とslam_toolboxをインストール
    ~~~
    git clone -b galactic https://github.com/ros-planning/navigation2.git
    rosdep install --from-paths src/navigation2
    git clone -b galactic https://github.com/SteveMacenski/slam_toolbox.git
    rosdep install --from-paths src/slam_toolbox
    colcon build --symlink-install
    ~~~

    3. navigation2を開始(ターミナルを４つ利用)
    ~~~
    cd ~/cugo_ws
    source install/setup.bash
    ros2 launch cugo_bringup robot.launch.py
    ~~~
    ~~~
    cd ~/cugo_ws
    source install/setup.bash
    ros2 run rplidar_ros rplidar_node
    ~~~

    ~~~
    cd ~/cugo_ws
    source install/setup.bash
    ros2 launch nav2_bringup navigation_launch.py
    ~~~
    ~~~
    cd ~/cugo_ws
    source install/setup.bash
    ros2 launch slam_toolbox online_async_launch.py
    ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
    ~~~


