启动激光雷达节点
ros2 launch start.py
启动相机节点
ros2 launch camera.launch.py    
加载环境
source setup_ros2.sh
采集数据
ros2 bag record -o data.bag /scan /image_raw
