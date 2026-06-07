# freshness_real_robot_logger

Minimal ROS2 `ament_python` package for topic-level real robot network calibration logging.

This package:

- subscribes to configured ROS2 topics
- extracts sender timestamps and sequence ids from headers or JSON payloads
- computes receiver-side inter-arrival, loss-gap, AoI, Effective Freshness, and bandwidth proxies
- writes CSV plus summary JSON / Markdown

This package does not:

- control robots
- change communication policy
- claim PHY/MAC measurements
- run Gazebo, shadow mode, or closed-loop execution
