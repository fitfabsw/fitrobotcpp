# New Launch (humble-multi)

## overview

All launch files are integrated into

### map_server

- map.launch.py (keepout or no-keepout)

  available arguments:

  - sim: true | false (default)
  - worldname
    - simulation: turtlebot3_world (default) | turtlebot3_house | room_with_tags | obstacles
    - real: fit_office_2f (default)
  - use_mask: true (default) | false
  - robot: linot2 (default) | artic

  Based on worlname and robot argument, the mapkey and the corresponding yaml file will be loaded.
  for example, if worldname is turtlebot3_world and robot is artic,
  the mapkey is turtlebot3_world/artic and the published topic is /turtlebot3_world/artic/map.

### bring-up

- real

  - robot.launch.py (multi or single)
    available arguments:
    - worldname

- simulation
  - sim.launch.py (single)
  - sim_multi.launch.py (multi)
    available arguments:
    - worldname

### navigation

- nav.launch.py (multi or single)
  available arguments:
  - worldname: turtlebot3_world | turtlebot3_house | room_with_tags | obstacles (needed)
  - sim: true | false (default)
  - rviz: true | false (default)
  - namespace: /robot1 (Optional, when robot namespace is needed)

### navigation w/ auto-docking

- navdock.launch.py (multi or single)

These files can handle all kinds of cases including

- single/multi robots
- keepout/no keepout
- simulation/realcase

## details

Below are the details of each launch file and separated into simulation and real cases.

### simulation

#### map_server

```bash
ros2 launch fitrobot map.launch.py sim:=true
or
ros2 launch fitrobot map.launch.py sim:=true worldname:=turtlebot3_world robot:=lino2 use_mask:=true
```

#### single robot

##### bringup

```bash
# lino2
ros2 launch linorobot2_gazebo sim.launch.py worldname:=turtlebot3_world
# artic (TBD)
ros2 launch articubot_one sim.launch.py worldname:=turtlebot3_world
```

##### navigation

```bash
# lino2
ros2 launch linorobot2_navigation nav.launch.py worldname:=turtlebot3_world sim:=true rviz:=true
or
ros2 launch linorobot2_navigation nav.launch.py worldname:=turtlebot3_world sim:=true rviz:=false
ros2 launch linorobot2_navigation rviz_launch.py worldname:=turtlebot3_world use_namespace:=true namespace:=/robot1
# artic (TBD)
ros2 launch articubot_one nav.launch.py worldname:=turtlebot3_world sim:=true rviz:=true
```

##### auto-docking

```bash
# lino2
LINOROBOT2_BASE=zbotlino2a ros2 launch linorobot2_gazebo sim.launch.py worldname:=room_with_tags
ros2 launch fitrobot map.launch.py sim:=true worldname:=room_with_tags
ros2 launch linorobot2_navigation navdock.launch.py sim:=true rviz:=true
ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot "{dock_id: 'home_dock_1'}"

ros2 launch linorobot2_navigation detect.launch.py image_remap:=camera/color/image_raw camera_info_remap:=camera/color/camera_info
```

#### multiple robots

##### bringup

```bash
# lino2
ros2 launch linorobot2_gazebo sim_multi.launch.py worldname:=turtlebot3_world
# artic
ros2 launch articubot_one sim_multi.launch.py worldname:=turtlebot3_world
```

##### navigation

```bash
# lino2
ros2 launch linorobot2_navigation nav.launch.py worldname:=turtlebot3_world sim:=true rviz:=true namespace:=/robot1
or
ros2 launch linorobot2_navigation nav.launch.py worldname:=turtlebot3_world sim:=true rviz:=false namespace:=/robot1
ros2 launch linorobot2_navigation rviz_launch.py worldname:=turtlebot3_world use_namespace:=true namespace:=/robot1
# artic (TBD)
ros2 launch articubot_one nav.launch.py sim:=true worldname:=turtlebot3_world rviz:=true namespace:=/robot1
```

##### auto-docking

```bash
# lino2
LINOROBOT2_BASE=zbotlino2a ros2 launch linorobot2_gazebo sim_multi.launch.py worldname:=room_with_tags
ros2 launch fitrobot map.launch.py sim:=true worldname:=room_with_tags
ros2 launch linorobot2_navigation navdock.launch.py sim:=true rviz:=true namespace:=/robot1
ros2 action send_goal /robot1/dock_robot opennav_docking_msgs/action/DockRobot "{dock_id: 'home_dock_1'}"
```

### real

#### map_server

##### with keepout zone (default)

```bash
ros2 launch fitrobot map.launch.py worldname:=fit_office_2f
```

##### without keepout zone

```bash
ros2 launch fitrobot map.launch.py worldname:=fit_office_2f use_mask:=false
```

#### multiple robots

##### bringup

```bash
# lino2
ros2 launch linorobot2_bringup robot.launch.py namespace:=/lino2
# artic
ros2 launch articubot_one robot.launch.py namespace:=/artic
```

##### navigation

- combine nav & rviz (run in pc)

```bash
# lino2
ros2 launch linorobot2_navigation nav.launch.py rviz:=true namespace:=/lino2
# artic
ros2 launch articubot_one nav.launch.py rviz:=true namespace:=/artic
```

- separate nav & rviz (nav run in car, rviz run in pc)

```bash
# lino2 (TBD)
ros2 launch linorobot2_navigation nav.launch.py namespace:=/lino2 # nav
ros2 launch linorobot2_navigation rviz_launch.py worldname:=fit_office_2f use_namespace:=true namespace:=/lino2 # rviz
# artic (TBD)
ros2 launch articubot_one nav.launch.py namespace:=/artic # nav
ros2 launch articubot_one rviz_launch.py worldname:=fit_office_2f use_namespace:=true namespace:=/artic # rviz
```

#### single robot

##### bringup

```bash
# lino2
ros2 launch linorobot2_bringup robot.launch.py
# artic
ros2 launch articubot_one robot.launch.py
```

##### navigation

- combine nav & rviz (run in pc)

  ```bash
  # lino2
  ros2 launch linorobot2_navigation nav.launch.py rviz:=true
  # artic
  ros2 launch articubot_one nav.launch.py rviz:=true
  ```

- separate nav & rviz (nav run in car, rviz run in pc)

  ```bash
  # lino2 (TBD)
  ros2 launch linorobot2_navigation nav.launch.py # nav
  ros2 launch linorobot2_navigation rviz_launch.py worldname:=fit_office_2f # rviz
  # artic (TBD)
  ros2 launch articubot_one nav.launch.py # nav
  ros2 launch articubot_one rviz_launch.py worldname:=fit_office_2f # rviz
  ```

##### auto-docking

```bash
# lino2
ros2 launch fitrobot map.launch.py worldname:=fit_office_2f
LINOROBOT2_BASE=zbotlino2a ros2 launch linorobot2_bringup robot.launch.py
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file $HOME/.ros/camera_info/usb_cam_params.yml
ros2 launch linorobot2_navigation navdock.launch.py rviz:=false # nav
ros2 launch linorobot2_navigation rviz_launch.py worldname:=fit_office_2f # rviz
ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot "{dock_id: 'home_dock_1'}"

```

# Testing New Launch (humble-multi)

## Lino2

### real

| checked | bring-up | map_server | navigation | rviz      |
| ------- | -------- | ---------- | ---------- | --------- |
| v       | multi    | keepout    | multi      | combined  |
| v       |          |            |            | separated |
| v       |          | no-keepout |            | combined  |
| v       |          |            |            | separated |
| v       | single   | keepout    | single     | combined  |
| v       |          |            |            | separated |
| v       |          | no-keepout |            | combined  |
| v       |          |            |            | separated |

### sim

| checked | bring-up | map_server | navigation | rviz     |
| ------- | -------- | ---------- | ---------- | -------- |
| v       | multi    | keepout    | multi      | combined |
| v       |          | no-keepout |            | combined |
| v       | single   | keepout    | single     | combined |
| v       |          | no-keepout |            | combined |

#### fitrobot services

##### services on central pc

Before using the services, you need to define ROBOT_INFO environment variable first.

ex: ROBOT_INFO="lino2:13a5;lino2:13a6"

- first robot
  robot_type: lino2
  robot_sn: 13a5
  robot_namespace: /lino2_13a5

- second robot
  robot_type: lino2
  robot_sn: 13a6
  robot_namespace: /lino2_13a6

These info will be used for sim.launch.py to generate the robots in gazebo.

The first robot_namespace is used for all the services for each robot.

###### bringup

ROBOT_INFO="lino2:13a5:0.0:0.5;artic:29f4:0.0:-0.5" ros2 launch fitrobot sim.launch.py

###### map

ros2 launch fitrobot map.launch.py sim:=true worldname:=turtlebot3_world use_mask:=true

##### services on each robot

###### check_robot_status

`ros2 run fitrobotcpp check_robot_status --ros-args -r __ns:=/lino2_13a5 -r /tf:=tf -r /tf_static:=tf_static`

###### master

ros2 run fitrobotcpp master_node --ros-args -p use_sim_time:=true

- services provided by master_node

  - call navigation service provided by master_node
    ros2 service call /lino2_ab12/navigation fitrobot_interfaces/srv/Navigation "{worldname: turtlebot3_world}"

    - rviz
      ros2 launch linorobot2_navigation rviz_launch.py worldname:=turtlebot3_world

  - call slam
    ros2 service call /lino2_ab12/slam fitrobot_interfaces/srv/Slam {}

    - rviz
      ros2 launch linorobot2_navigation rviz_launch.py

  - call termination
    ros2 service call /lino2_ab12/terminate_slam_or_navigation fitrobot_interfaces/srv/TerminateProcess {}

## artic

### real

| checked | bring-up | map_server | navigation | rviz      |
| ------- | -------- | ---------- | ---------- | --------- |
| v       | multi    | keepout    | multi      | combined  |
| v       |          |            |            | separated |
| v       |          | no-keepout |            | combined  |
| v       |          |            |            | separated |
| v       | single   | keepout    | single     | combined  |
| v       |          |            |            | separated |
| v       |          | no-keepout |            | combined  |
| v       |          |            |            | separated |

### sim

| checked | bring-up | map_server | navigation | rviz     |
| ------- | -------- | ---------- | ---------- | -------- |
| v       | multi    | keepout    | multi      | combined |
| v       |          | no-keepout |            | combined |
| v       | single   | keepout    | single     | combined |
| v       |          | no-keepout |            | combined |
