# ros2_buildhat_differential_drive
ROS2 Differential Drive Controller for Raspberry Pi Build HAT (a hardware connector for Lego Spike Technic)

## Docker Install (for Brian the demo robot)

For a docker install:

```
docker build -t brian docker/
```

To run:
```
docker run -it --rm --net=host --ipc=host --device /dev/ttyAMA0 brian
```
