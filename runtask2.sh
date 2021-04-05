
gnome-terminal -- roslaunch husky_gazebo husky_playpen.launch gui:=false
gnome-terminal -- roslaunch husky_navigation move_base_mapless_demo.launch
catkin build
source devel/setup.bash -i
rosrun task2 csvgoals.py
rosrun task2 csvgoals