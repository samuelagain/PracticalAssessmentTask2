# Practical Assessment Task 2

The following demo is designed to run on Ubuntu 18.04 LTS

In order to run the attached demo the following packages are required:

- ROS Melodic (Found Here: http://wiki.ros.org/melodic/Installation/Ubuntu)

- Husky Gazebo Simulator (Found Here: http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky)

# Install

Copy the directory ```task1``` into the ```/src``` folder of your catkin workspace.

If you do not have a catkin workspace instruction for istallation can be found here (http://wiki.ros.org/catkin)


# Usage

1. Launch the Husky Gazebo Environment:

``` roslaunch husky_gazebo husky_playpen.launch ```

2. In a seperate terminal, launch the Husyky Move Base Server Environment:

``` roslaunch husky_navigation move_base_mapless_demo.launch ```

3. Navigate to your catkin workspace. Using Terminal:

  ```cd <your-catkin-workspace>```

4. Build the ros nodes:

```catkin build```

6. Add the source to the package:

```source devel/setup.bash```

8. Run the goal seeking specific node you seek.

For the python version:
```rosrun task2 csvgoals.py```

For the c++ version:
```rosrun task2 csvgoals```

