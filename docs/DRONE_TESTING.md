# BeamNG ROS 2 Drone – Full Testing Syntaxes

Replace `172.23.64.1` with your Windows host IP (from `ipconfig` → `vEthernet (WSL)`).

## Prerequisites

- BeamNG.tech running with `-tcom-listen-ip 0.0.0.0` or `-tcom-listen-ip 172.23.64.1`
- ROS 2 workspace built and sourced: `source install/setup.bash`

---

## All-in-one launch (bridge + scenario + controller + cmd_vel converter)

```bash
ros2 launch beamng_bringup beamng_drone_all.launch.py host:=172.23.64.1 port:=25252 scenario:=/config/scenarios/drone_scenario.json vehicle_id:=drone
```

With defaults (host=172.23.64.1, port=25252):

```bash
ros2 launch beamng_bringup beamng_drone_all.launch.py
```

Override any arg:

```bash
ros2 launch beamng_bringup beamng_drone_all.launch.py host:=192.168.1.111 port:=25252
```

If the drone controller times out, the scenario may need more time to load. Increase the delay:

```bash
ros2 launch beamng_bringup beamng_drone_all.launch.py controller_delay:=25.0
```

Then run keyboard teleop in a separate terminal:

```bash
ros2 run beamng_teleop_keyboard drone_teleop
```

---

## Setup (run in order, alternative to all-in-one)

### Terminal 1 – Bridge

```bash
ros2 run beamng_ros2 beamng_bridge --ros-args -p host:=172.23.64.1 -p port:=25252
```

### Terminal 2 – Start drone scenario

```bash
ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario "{path_to_scenario_definition: '/config/scenarios/drone_scenario.json'}"
```

### Terminal 3 – Drone controller

```bash
ros2 run beamng_agent drone_controller --ros-args -p host:=172.23.64.1 -p port:=25252 -p vehicle_id:=drone
```

### Terminal 4 – Keyboard teleop (interactive)

```bash
ros2 run beamng_teleop_keyboard drone_teleop
```

### Optional – cmd_vel converter

```bash
ros2 launch beamng_bringup beamng_drone_teleop
```

---

## Verification / testing

### Check nodes

```bash
ros2 node list
ros2 node info /beamng_bridge
ros2 node info /beamng_drone_controller
```

### Check topics

```bash
ros2 topic list
ros2 topic list | grep -E 'drone|cmd_vel|control'
```

### Echo control traffic

```bash
ros2 topic echo /drone_control
ros2 topic echo /cmd_vel
```

### Check services

```bash
ros2 service list | grep beamng
```

### Check parameters

```bash
ros2 param list /beamng_bridge
ros2 param list /beamng_drone_controller
ros2 param get /beamng_drone_controller host
```

### Manual publish (no teleop)

```bash
ros2 topic pub /drone_control beamng_msgs/msg/DroneControl "{ascend: 0.5, descend: 0.0, pitch: 0.0, roll: 0.0, yaw: 0.0, yaw_left: 0.0, yaw_right: 0.0, takeoff_land: false, toggle_mode: false}" --once
```

---

## Drone teleop keys

| Key | Action        |
|-----|---------------|
| w/x | Ascend/descend|
| a/d | Roll left/right |
| q/e | Yaw left/right |
| i/k | Pitch forward/backward |
| t   | Takeoff/land toggle |
| m   | Flight mode toggle |
| s / space | Hover (stop) |
