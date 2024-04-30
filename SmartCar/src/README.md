# SmartCar
#author:Sea of horse
#date:2024/04/21
#email:2042312865@qq.com

Gazebo 3D simulator example of ROS tutorials (version: Noetic)
Tested on Ubuntu 20.04


**## Getting Started**

1. Create a project workspace
***可省略此步***
\```bash

***mkdir -p ~/SmartCar/src***

\```

2. catkin make the project

\```bash

***cd ~/SmartCar***

***catkin_make***

\```

3. Setup .bashrc

\```bash

***source devel/setup.bash***

\```

4. Start the gazebo and rviz

\```bash

***roslaunch mbot_gazebo mbot_gazebo_twobot.launch***

\```

5. Start control mbot1 move and mbot2 follow

\```bash

***roslaunch mbot_gazebo multi_robot_control.launch***

\```
