## Robot Arm Mover Simulation With ROS


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


## ROS Nodes
<p align="justify">
ROS breaks complex steps into many small Unix processes called nodes. Typically each node on the system is responsible for one small and relatively specific portion of the robot's overall functionality. Here, in our simulation, three nodes are used to achieve safe operation of the robot.  </p>

- simple_mover-publish joint angle to simple_arm
- arm_mover-provides a service called safe_mode
- safe_move-allows the arm to be moved to any position within its workspace. The safe zone is bounded my minimum and maximum joint angles, and is configurable via the ROS' parameter server

##  ROS Master Process
<p align="justify">
At the center of these collections of nodes is ROS Master Process which acts as a sort of manager of all the nodes. The ROS Master maintains a registry of all the active nodes on a system. It then allows each node to discover other nodes in the system and establish lines of communication with them. In addition to allowing nodes to locate one another and communicate, the ROS master also hosts what's called the parameter server. </p>

![ros](https://user-images.githubusercontent.com/37708330/53758563-60305780-3ebe-11e9-962d-34ea8fce9850.png)


## ROS Publishers
<p align="justify">
Publishers allow a node to send messages to a topic, so that data from the node can be used in other parts of the ROS system. In Python, ROS publishers typically have the following definition format, although other parameters and arguments are possible: </p>

```
pub1 = rospy.Publisher("/topic_name", message_type, queue_size=size)
```
![publishers](https://user-images.githubusercontent.com/37708330/53758673-b1404b80-3ebe-11e9-93bb-2d64de41fc02.png)

The "/topic_name" indicates which topic the publisher will be publishing to. The message_type is the type of message being published on "/topic_name".

ROS publishing can be either synchronous or asynchronous:

- Synchronous publishing means that a publisher will attempt to publish to a topic but may be blocked if that topic is being published to by a different publisher. In this situation, the second publisher is blocked until the first publisher has serialized all messages to a buffer and the buffer has written the messages to each of the topic's subscribers. This is the default behavior of a rospy.Publisher if the queue_size parameter is not used or set to None.
- Asynchronous publishing means that a publisher can store messages in a queue until the messages can be sent. If the number of messages published exceeds the size of the queue, the oldest messages are dropped. The queue size can be set using the queue_size parameter.
Once the publisher has been created as above, a message with the specified data type can be published as follows:
```
pub1.publish(message)
```

## ROS Subscribers
<p align="justify">
A Subscriber enables your node to read messages from a topic, allowing useful data to be streamed into the node. In Python, ROS subscribers frequently have the following format, although other parameters and arguments are possible: </p>

```
sub1 = rospy.Subscriber("/topic_name", message_type, callback_function)
```
<p align="justify">
The "/topic_name" indicates which topic the Subscriber should listen to.The message_type is the type of message being published on "/topic_name". The callback_function is the name of the function that should be called with each incoming message. Each time a message is received, it is passed as an argument to callback_function. Typically, this function is defined in your node to perform a useful action with the incoming data. Note that unlike service handler functions, the callback_function is not required to return anything. </p>


## ROS Services
<p align="justify">
A ROS service allows request/response communication to exist between nodes. Within the node providing the service, request messages are handled by functions or methods. Once the requests have been handled successfully, the node providing the service sends a message back to the requester node. In Python, a ROS service can be created using the following definition format:
</p>  

```
service = rospy.Service('service_name', serviceClassName, handler)
```

<p align="justify">
On the other hand, to use a ROS service from within another node, you will define a ServiceProxy, which provides the interface for sending messages to the service: </p>  

```
service_proxy = rospy.ServiceProxy('service_name', serviceClassName)
msg = serviceClassNameRequest()
#update msg attributes here to have correct data
response = service_proxy(msg)
```

### Creating ROS Services

Interaction with a service consists of two messages being passed. A request passed to the service, and a response received from the service. The definitions of the request and response message type are contained within .srv files living in the srv directory under the package’s root.


## Some Essential ROS commands
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



###  roscd log

###  rostopic echo /rosout

###  rospy.init_node('my_node', log_level=rospy.DEBUG)

