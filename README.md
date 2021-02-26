# Реализация алгоритмов компьютерного зрения и искуственного интеллекта с использованием платформы NVIDIA Jetson
ROS2 + NVIDIA Jetson + Intel RealSense

## Задачи
1. Исследование сфер применения платформы NVIDIA Jetson
- Skydio
- AlphaPilot AI
2. Тестирование взаимодействия с RealSense
- Обработка изображения
- обнаружение объектов
- стерео зрение
3. Обеспечение работы с использованием ROS2 (Наладка зависимостей и версий, обновление пакетов)
4. Решение типичных задач c использованием компьютерного зрения и нейронных сетей
5. Внедрение полученных результатов для позиционирования робота-художника и построения траекторий движения рабочего инструмента.

## How to get started
Run RealSense node in ROS2
```
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/local_setup.bash
ros2 launch realsense2_camera rs_launch.py
```

## Published Topics
- /camera/accel/imu_info
- /camera/color/camera_info
- /camera/color/image_raw
- /camera/depth/camera_info
- /camera/depth/color/points
- /camera/depth/image_rect_raw
- /camera/extrinsics/depth_to_color
- /camera/extrinsics/depth_to_infra1
- /camera/extrinsics/depth_to_infra2
- /camera/gyro/imu_info
- /camera/imu
- /camera/infra1/camera_info
- /camera/infra1/image_rect_raw
- /camera/infra2/camera_info
- /camera/infra2/image_rect_raw
- /camera/parameter_events
- /camera/rosout
- /parameter_events
- /rosout
- /tf_static

## Depth camera test
![img1](/photos/Screenshot from 2021-02-26 13-44-09.png)
