# 容器管理
.PHONY: build run shell up down logs clean rebuild

build:
	docker compose build

# 推荐：前台进入容器
shell:
	docker compose run --rm ros2 bash

# 后台启动
up:
	docker compose up -d ros2

# 进入后台容器
exec:
	docker compose exec ros2 bash

down:
	docker compose down

logs:
	docker compose logs -f ros2

# 清理（容器 + 镜像 + 编译产物 + 数据）
clean:
	docker compose down --rmi all
	rm -rf bags/* ros2_ws/build ros2_ws/install ros2_ws/log
	@echo "已清理（注意：rslidar_sdk / rslidar_msg / camera_driver / key_save 源码保留）"

# 仅清理 colcon 产物，触发重新编译
rebuild:
	rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
	@echo "已清理编译产物，colcon build 即可重编"

# 在容器内编译（先用 shell 进入）
build-ws:
	docker compose run --rm ros2 bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select rslidar_msg rslidar_sdk camera_driver key_save"

# 在容器内运行一键 launch
launch-all:
	docker compose run --rm ros2 bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 launch key_save all_em4.launch.py"
