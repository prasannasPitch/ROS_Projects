## ROS Projects


### Setting up CATKIN Workspace

All of the ROS related code developed throughout this repositiory will reside in your catkin workspace. We only need to create and initialize the workspace once.

#### Step 1: mkdir -p ~/catkin_ws/src
<p align="justify">
First, create the top level catkin workspace directory and a sub-directory named src (pronounced source). The top level directory’s name is arbitrary, but is often called catkin_ws (an abbreviation of catkin_workspace), so we will follow this convention. You can create these two directories with a single command:  </p>

```
$ mkdir -p ~/catkin_ws/src
```

####  Step 2: cd ~/catkin_ws/src
Next, navigate to the src directory with the cd command:

```
$ cd ~/catkin_ws/src
```

####  Step 3: catkin_init_workspace
Now you can initialize the catkin workspace:

```
$ catkin_init_workspace
```
Notice that a symbolic link (CMakeLists.txt) has been created to /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake

####  Step 4: cd ~/catkin_ws
Return to the top level directory
```
$ cd ~/catkin_ws
```

#### Step 5: catkin_make
and build the workspace.

Note: you must issue this command from within the top level directory (i.e., within catkin_ws NOT catkin_ws/src)

```
$ catkin_make
```
<p align="justify">
You now have two new directories: build and devel. The aptly named build directory is the build space for C++ packages and, for the most part, you will not interact with it. The devel directory does contain something of interest, a file named setup.bash. This setup.bash script must be sourced before using the catkin workspace.  </p>


### roslaunch

roslaunch allows you to do the following

- Launch ROS Master and multiple nodes with one simple command
- Set default parameters on the parameter server
- Automatically re-spawn processes that have died

To use roslaunch, you must first make sure that your workspace has been built, and sourced.

### rosdep

ROS packages have two different types of dependencies: build dependencies, and run dependencies. The rosdep tool will check for a package's missing dependencies, download them, and install them. To check for missing dependencies in the simple_arm package:
```
$ rosdep check package_name
```
Note: In order for the command to work, the workspace must be sourced. This gives you a list of the system dependencies that are missing, and where to get them. To have rosdep install packages, invoke the following command from the root of the catkin workspace
