# Upute za pokretanje

1. Potrebno je imati Linux ^20.04 instaliran (ROS trenutno podržava samo Linux OS)
2. Instalirajte ROS Noetic ili višlju verziju
3. Instalirajte potrebne pakete sudo apt-get update && sudo apt-get install -y && ros-{ros-verzija}-joint-state-publisher-gui && ros-{ros-verzija}-gazebo-ros && ros-{ros-verzija}-xacro && ros-{ros-verzija}-ros2-control && ros-{ros-verzija}-moveit && ros-{ros-verzija}-ros2-controller && ros-{ros-verzija}-gazebo-ros2-control
4. Instalirajte potrebne C++ i Python biblioteke sudo apt-get update && sudo apt-get install -y libserial-dev python3-pip
5. pip install pyserial
6. Kloniraje repozotorij - git clone https://github.com/maurosterpin/ros-robot-arm.git
7. Unutar ros-robot-arm foldera pokrenite komandu u terminalu catkin_make da se izgradi projekt
8. Pokrenite source/devel setup.bash kako bi ROS prepoznao environment projekta
9. Pokrenite projekt sa skriptom - roslaunch robot_bringup complete.launch 
