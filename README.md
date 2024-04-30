#ROS practice ported to ROS Noetic & Gazebo 11

***.:: First version, please tell me the issues or help me to fix it ::.***

## Requirements

I. You need the following packages before install `Robot_practice`.


II. Build.
```sh
cd ~/catkin_ws && catkin_make
```

III. Clone `Robot_practice`.
```sh
git clone https://github.com/HaiDongYYDS/Robot_practice.git
```
IV. Setup .bashrc
```sh
source devel/setup.bash
```
V. Repeat step II.

## Usage

* Start the gazebo and rviz:
    ```sh
    roslaunch mbot_gazebo mbot_gazebo_twobot.launch
    ```
* Start control mbot1 move and mbot2 follow:
    ```sh
   roslaunch mbot_gazebo multi_robot_control.launch
    ```
