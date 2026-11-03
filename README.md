# APF+ TurtleBot3 Burger

Current direct-LiDAR APF controller with temporary goals and path memory, for ROS 2 Jazzy. No prior map or SLAM is used for navigation. This is experimental hardware-control code; escape from arbitrary U-shaped obstacles is not guaranteed.

## Source files

- `reactive_controller.py`: current ROS controller, goals, arming, logging.
- `reactive_core.py`: APF steering, temporary targets, stagnation and footprint checks.
- `drive_relay.py`: robot-local motor-command relay and command timeout.
- `drive_safety.py`: shared speed limits and command lease.
- `live_controller.py`, `live_preview.py`, `live_core.py`: required shared sensor/TF, control and utility dependencies. Their older standalone navigation entry points are not the selected controller.
- `trajectory_view.py`: optional odometry trajectory display.

Only current runtime source and its dependencies are included. Offline simulations, earlier standalone prototypes, trial logs and figures are excluded. This snapshot was copied from the laptop; the robot copy has not been compared because its hostname was unavailable during export. TurtleBot3 bringup and drivers are external installed dependencies, not included source.

## Dependencies

ROS 2 Jazzy with rclpy, tf2_ros, sensor_msgs, geometry_msgs, nav_msgs, visualization_msgs, std_msgs and std_srvs; Python numpy and scipy.

## Run

In each ROS terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
```

On the robot, keep bringup running:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

In a separate robot terminal, from this source directory:

```bash
python3 drive_relay.py
```

In another terminal on the robot (or a sourced laptop with all dependencies):

```bash
python3 reactive_controller.py --ros-args -p max_linear:=0.03 -p max_angular:=0.2 -p margin:=0.02 -p goal_obstacle_clearance:=0.02
```

Do not run another controller or teleop publishing motor commands alongside it. The controller starts disarmed. On the laptop, run `rviz2`, set Fixed Frame to `odom`, and add LaserScan `/scan`, Path `/apf/live_path`, and MarkerArray `/apf/live_goals`.

Arm only when ready for real motion, then choose a NEW free-space 2D Goal Pose:

```bash
ros2 service call /apf/arm std_srvs/srv/Trigger '{}'
```

Stop/disarm:

```bash
ros2 service call /apf/stop std_srvs/srv/Trigger '{}'
```

Keep access to the physical stop. LiDAR blind spots, odometry drift and sampled footprint prediction limit collision checks. A failed relay cannot itself enforce a stop.

Optional trajectory visualization on laptop:

```bash
python3 trajectory_view.py
```

Add Path `/apf/actual_path` in RViz. This is an odometry trace, not independently measured ground truth.
