#ROS practice ported to ROS Noetic & Gazebo 11

***.:: First version, please tell me the issues or help me to fix it ::.***

## Requirements

I. You need the following packages before install `robot_practice`.

* unique_identifier:
    ```sh
    git clone https://github.com/ros-geographic-info/unique_identifier.git
    ```
* geographic_info:
    ```sh
    git clone https://github.com/ros-geographic-info/geographic_info.git
    ```
II. Build.
```sh
cd ~/catkin_ws && catkin_make
```

III. Clone `hector_quadrotor_noetic`.
```sh
git clone https://github.com/RAFALAMAO/hector_quadrotor_noetic.git
```

IV. Repeat step II.

## Usage

Run a simulation by executing the launch file in `hector_quadrotor_gazebo` and `hector_quadrotor_demo` packages (only these work at the momment, but you can try other ones):

* Run the following for one drone inside an empty world:
    ```sh
    roslaunch hector_quadrotor_gazebo quadrotor_empty_world.launch
    ```
* Run the following for one dron outdoor:
    ```sh
    roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch
    ```
* Run the following for one dron outdoor without rviz interface:
    ```sh
    roslaunch hector_quadrotor_demo outdoor_flight_gazebo_no_rviz.launch
    ```
* Run the following for two drones inside an empty world:
    ```sh
    roslaunch hector_quadrotor_demo two_drones_empty.launch
    ```
