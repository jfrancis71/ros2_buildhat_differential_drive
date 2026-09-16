# ros2_buildhat_differential_drive
ROS2 Differential Drive Controller for Raspberry Pi Build HAT (a hardware connector for Lego Spike Technic)

<img src=./brian/images/front.jpg width=300> <img src=./brian/images/back.jpg width=300>

### Tested Hardware

Raspberry Pi 5 (8GB), Raspberry Pi Foundation BuildHat

### Tested Software

Ubuntu 24.04, ROS2 Jazzy, Raspberry Pi Foundation BuildHat (Python version)

## Docker Install (for Brian the demo robot)

For a docker install:

```
docker build -t brian docker/
```

To run:
```
docker run -it --rm --net=host --ipc=host --device /dev/ttyAMA0 brian
```

### Subscribed Topics

- /cmd_vel

### Published Topics

- /odom

### Published Transforms

- odom->base_link


### Parameters

Similar to the ROS2 Control Differential Drive Controller

- wheel_radius: radius of the wheels
- wheel_seperation: distance (not radius) between the two main wheels
- cmd_vel_timeout: timeout after which if no message received, motors stop
- left_wheel_name: lego port name of left wheel
- right_wheel_name: lego port name of right wheel
- left_wheel_radius_multiplier: multiplier for left wheel radius (used in case of gearing)
- right_wheel_radius_multiplier: multiplier for right wheel radius (used in case of gearing)
