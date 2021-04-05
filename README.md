# Practical Assessment Task 2

This a demostration of a program that reads a CSV file into coordinates and directs a simulated robot via ROS to navigate to those coordinates. The results of this movement and navigation are then recorded in the csv file.

The following demo is designed to run on Ubuntu 18.04 LTS

In order to run the attached demo the following packages are required:

- ROS Melodic (Found Here: http://wiki.ros.org/melodic/Installation/Ubuntu)

- Husky Gazebo Simulator (Found Here: http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky)

# Install

Copy the directory ```task2``` into the ```/src``` folder of your catkin workspace.

If you do not have a catkin workspace, instruction for installation can be found here (http://wiki.ros.org/catkin)

# Configuring and reading the CSV file
Coordinates are to be entered in the csv file with the first column being the x coordinate and the second column being the y coordinate.
The next coordinate is stored on the new line.
The result of the navigation to each respective coordinate is stored in the third column adjacent to its relevant coordinate.

The file is to be always called
```coordinates.csv```

It is to be kept directly in the catkin workspace and not any subfolders.


# Usage

1. Launch the Husky Gazebo Environment:

``` roslaunch husky_gazebo husky_playpen.launch ```

2. In a seperate terminal, launch the Husyky Move Base Server:

``` roslaunch husky_navigation move_base_mapless_demo.launch ```

3. Navigate to your catkin workspace.

  ```cd <your-catkin-workspace>```

4. Build the ros nodes:

```catkin build```

6. Add the source to the package:

```source devel/setup.bash```

8. Run the csv-goal-seeking node.

For the python version:

```rosrun task2 csvgoals.py```

For the c++ version:

```rosrun task2 csvgoals```

