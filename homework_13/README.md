# ДЗ 13: antidrone_turret — рішення

## Опис


Додав 3 додаткові ноди - для контролера, гімбала і серво, а також їх відповідні 3 меседжі. Логіка написана чисто на сі, аби не було депенденсу на рос. Нода читає повідомлення, робить логіку і преедає результат в рос.
## Схема

```text
target_track_publisher_node --/perception/target--> turret_controller_node
actuator_node               --/actuator/status---> turret_controller_node

turret_controller_node --/gimbal/cmd------> gimbal_driver_node
turret_controller_node --/servo/cmd-------> yaw_servo_driver_node
turret_controller_node --/actuator/trigger (сервіс)--> actuator_node
turret_controller_node --/turret/status---> ros2 topic echo
```

## Build / test

```bash
source /opt/ros/jazzy/setup.bash
cd homework_13/robot_ws
colcon build --symlink-install --packages-select antidrone_turret
source install/setup.bash
colcon test --packages-select antidrone_turret
colcon test-result --verbose
```

`Summary: 25 tests, 0 errors, 0 failures, 0 skipped` (16 наданих + 9 тестів логіки контролера).

## Запуск

```bash
ros2 launch antidrone_turret system.launch.py track:=approach_trigger.csv
```

## Результати

```bash
$ ros2 topic echo /turret/status --once
target_state: 2        # TARGET_LOCKED
action: 1              # ACTION_TRACK
trigger_state: 2       # TRIGGER_RELOADING
confidence: 0.9599999785423279
distance_m: 22.0
---
$ ros2 topic echo /gimbal/cmd --once
direction: 1           # UP
target_y: 190.0
error_y: 50.0
---
$ ros2 topic echo /servo/cmd --once
direction: 1           # RIGHT
target_x: 430.0
error_x: 110.0
---
$ ros2 topic echo /actuator/status --once
state: 0               # READY
trigger_count: 12
---
```

## Висновок

Трігер викликаєтсья тільки у випадку коли актуатор у реді. Це гарантовано decideTrigger
```text
28m conf=0.91 -> trigger accepted, trigger_count=1
26m conf=0.92 -> наведення є, пострілу немає
20m conf=0.94 -> наведення є, пострілу немає
15m conf=0.95 -> наведення є, пострілу немає
11m conf=0.96 -> наведення є, пострілу немає
 7m conf=0.96 -> наведення є, пострілу немає
[actuator ready after reload]
```

П'ять кадрів підряд у межах `max_distance_m=30` з `confidence` вище порога — жодного повторного виклику сервісу, бо актуатор був у `RELOADING`. Echo `/turret/status` вище зафіксував саме такий кадр: `TARGET_LOCKED` + `ACTION_TRACK` + `TRIGGER_RELOADING` на 22 м.