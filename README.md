# IRS-3_NIU-VSHE-2026
IRS-3 - Intelligent (mobile) robotic systems

## Overview
Проект мобильного робота на ROS 2 (Jazzy). Основные блоки:
- управление движением через `cmd_vel` и Arduino
- камеры RealSense + YOLO (по запросу через `camera_run`)
- лидар RPLidar C1 и анализатор стен

## Packages and Nodes
### arduinoDriver
Node: `arduinoDriver`
- Subscribes: `cmd_vel` (`geometry_msgs/Twist`), `servo_angle_1` (`std_msgs/Int32`), `servo_angle_2` (`std_msgs/Int32`)
- Publishes: `encoders` (`std_msgs/Int32MultiArray`, 4 числа), `gy25` (`std_msgs/Int32`)
- Протокол к Arduino:
  - Команда движения: `v <forward> <left> <rotation>`
  - Команда серв: `A <servo1> <servo2>`
  - Вход от Arduino: `ENC: e1 e2 e3 e4 GY25: angle`

### main
Nodes:
- `main` — концептуальный алгоритм движения по дороге с учетом `cam_date`
- `minimal` — алгоритм по строке `target_way`, включает `camera_run` между шагами
- `minimal_2` — поворот относительно стартового угла гироскопа
- `RC` — управление с клавиатуры (TTY, без GUI)
- `RC_X` — управление с клавиатуры (GUI, `pynput`)
- `test` / `main_test_motor_node` — тестовые ноды для `cmd_vel` и сенсоров

### realSenceCam
Nodes:
- `realSenceCam` — запись RGB видео с RealSense (по кнопке)
- `realSenceCam_road` — RealSense + YOLO
  - Subscribes: `camera_run` (`std_msgs/Int32`, 0/1)
  - Publishes: `cam_date` (`std_msgs/String`)
  - Формат `cam_date`: `label:x1,y1,x2,y2;label2:x1,y1,x2,y2`
  - Камера инициализируется всегда, обработка кадров только при `camera_run=1`
  - Лог в терминал: детекции с bbox + периодический `DETECT bus_stop` (раз в 20 сек при активной камере)

### lidarAnalyzer
Nodes:
- `lidarAnalyzer_node` — анализ стен по 4 секторам + EMA фильтр
- `lidarAnalyzer_service` — упрощенная версия без фильтра
Subscribes: `RplidarC1` (`sensor_msgs/LaserScan`)  
Publishes: `lidarAnalyzer` (`sensor_msgs/LaserScan`)  
Формат `lidarAnalyzer.ranges`:
`[front_dist, front_angle, left_dist, left_angle, right_dist, right_angle, back_dist, back_angle]`  
`intensities`: 1.0 если направление валидно, иначе 0.0

### RplidarC1
Node: `RplidarC1`
- Publishes: `RplidarC1` (`sensor_msgs/LaserScan`)

## Topics
- `cmd_vel` (`geometry_msgs/Twist`):  
  `linear.x` — вперед/назад, `linear.y` — влево/вправо, `angular.z` — поворот
- `encoders` (`std_msgs/Int32MultiArray`): 4 значения энкодеров
- `gy25` (`std_msgs/Int32`): угол гироскопа
- `servo_angle_1`, `servo_angle_2` (`std_msgs/Int32`): углы серв
- `camera_run` (`std_msgs/Int32`): 0/1 — включение обработки камеры
- `cam_date` (`std_msgs/String`): список детекций `label:x1,y1,x2,y2;...`
- `RplidarC1` (`sensor_msgs/LaserScan`): сырые данные лидара
- `lidarAnalyzer` (`sensor_msgs/LaserScan`): анализ 4 секторов

## Commands (setup / build / run)
```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bash
source install/setup.bash

colcon build

ros2 run realSenceCam realSenceCam_road --ros-args -p model_path:=/home/ubuntu/IRS-3_NIU-VSHE-2026/realSenceCam/realSenceCam/best.pt
ros2 run arduinoDriver arduinoDriver --ros-args -p port:=/dev/ttyUSB0
ros2 run main minimal --ros-args -p target_way:=ffrfffsssfrffrfff
```

## problems:
- /dev/USB0-1 - lidar|arduino  - путаются
- сделать копию образа sd raspberry
- добавить автоматическое отключение ардуинки при отсутствии команд
- необходим роутер
- Оптическая развязка
- Ардуино мега про мини и плата расширения для нее - как управляющий контроллер
- Логирование + логи в тхт файл
- нормальное определение знаков по видео
- Добавить отключение ардуино, если в на нее не приходят команды отключения (когда топик отключается), или другое аварийное отключение
- Плата шилда для управления на 4 мотора
- Питание от повербанка
