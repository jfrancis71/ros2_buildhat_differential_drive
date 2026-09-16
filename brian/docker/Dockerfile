FROM ros:jazzy-ros-base
RUN apt update
RUN apt install -y vim
RUN apt install -y python3-pip
RUN apt install -y ros-jazzy-tf-transformations
RUN pip install --break-system-packages buildhat
WORKDIR /root/ros2_ws
RUN mkdir src
RUN git -C src clone https://github.com/jfrancis71/ros2_buildhat_differential_drive.git
RUN colcon build --symlink-install --packages-select=buildhat_differential_drive
RUN colcon build --symlink-install --packages-select=brian
CMD /bin/bash -c "ln -s /dev/ttyAMA0 /dev/serial0; source ./install/setup.bash; ros2 launch brian motors_launch.py"
