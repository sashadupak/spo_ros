# Позиционирование для робота-художника
## Описание
Приложение публикует в ros2 топик /result список из 4 пар координат холста в локальной системе координат 

## Технологии
* ROS2
* NVIDIA Jetson nano
* Intel RealSense

## Задачи проекта
1. Исследование сфер применения платформы NVIDIA Jetson
2. Тестирование взаимодействия с RealSense
3. Обработка изображения
4. Обнаружение объектов
5. Обеспечение работы с использованием ROS2 (Наладка зависимостей и версий, обновление пакетов)
6. Внедрение полученных результатов для позиционирования робота-художника .

## Пререквизиты
* ROS2
* [ROS2 realsense wrapper](https://github.com/IntelRealSense/realsense-ros)
* Python (numpy, opencv)

## Установка
Установите зависимости для python
```
pip install -r requirements.txt
```
Клонируйте репозиторий в папку src в своем ROS2 рабочем пространстве
```
cd ~/ros2_ws/src
git clone https://github.com/sdupak/spo_ros
```
Установите необходимые зависимости
```
cd ..
rosdep install -i --from-path src --rosdistro <distro> -y
```
Соберите и актуализируйте директории, где ROS ищет установленные пакеты
```
colcon build --packages-select mypack
source install/setup,bash
```


## Запуск
Запустите ноду realsense со следующими параметрами
```
ros2 launch realsense2_camera rs_launch.py enable_pointcloud:=true align_depth:=true filters:=spatial ordered_pc:=true device_type:=d435i
```
Запустите приложение
```
ros2 run mypack start
```

## [Ноды и публикуемые топики:](https://ilvif666.github.io/)
![alt text](https://raw.githubusercontent.com/sdupak/spo_ros/8bbf452c9503f4b53fb9f7da07095b3bf7737512/photos/rqt_graph.png)
## 
