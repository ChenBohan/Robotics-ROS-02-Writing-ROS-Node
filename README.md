# Robotics-ROS-02-Writing-ROS-Node
Udacity Self-Driving Car Engineer Nanodegree: Writing ROS Node

## ROS Publishers

```python
pub1 = rospy.Publisher("/topic_name", message_type, queue_size=size)
```
Once the publisher has been created as above, a message with the specified data type can be published as follows:

```python
pub1.publish(message)
```

- ROS publishing can be either synchronous or asynchronous:
  - Synchronous publishing means that a publisher will attempt to publish to a topic but may be blocked if that topic is being published to by a different publisher.
  - Asynchronous publishing means that a publisher can store messages in a queue until the messages can be sent.
  
## ROS node (Simple Mover)

### Adding the scripts directory

```bash
cd ~/catkin_ws/src/simple_arm/
mkdir scripts
```

### Creating a new script

```bash
cd scripts
touch simple_mover
chmod u+x simple_mover
```
After setting the appropriate execution permissions on the file, rebuilding the workspace, and sourcing the newly created environment, you will be able to run the script.

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rosrun simple_arm simple_mover
```

### The Code

```python
def mover():
    pub_j1 = rospy.Publisher('/simple_arm/joint_1_position_controller/command',
                             Float64, queue_size=10)
    pub_j2 = rospy.Publisher('/simple_arm/joint_2_position_controller/command',
                             Float64, queue_size=10)
    rospy.init_node('arm_mover')
    rate = rospy.Rate(10)
    start_time = 0

    while not start_time:
        start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        elapsed = rospy.Time.now().to_sec() - start_time
        pub_j1.publish(math.sin(2*math.pi*0.1*elapsed)*(math.pi/2))
        pub_j2.publish(math.sin(2*math.pi*0.1*elapsed)*(math.pi/2))
        rate.sleep()
```
- ``queue_size`` parameter is used to determine the maximum number messages that may be stored in the publisher queue before messages are dropped.
- ``init_node()`` must be called before any other rospy package functions are called. 
- ``anonymous=True`` makes sure that you always have a unique name for your node
- ``rate`` is used to limit the frequency at which certain loops spin in ROS.
- ``rospy.Time.now()`` will initially return 0, until the first message has been received on the ``/clock`` topic. 
- If the name variable is set to ``main``, indicating that this script is being executed directly, the ``mover()`` function will be called. The try/except blocks here are significant as rospy uses exceptions extensively. The particular exception being caught here is the ``ROSInterruptException``. This exception is raised when the node has been signaled for shutdown. If there was perhaps some sort of cleanup needing to be done before the node shuts down.

## ROS Services
### Defining services

A ROS service allows request/response communication to exist between nodes.

```python
service = rospy.Service('service_name', serviceClassName, handler)
```
- ``service_name`` is the name given to the service. Other nodes will use this name to specify which service they are sending requests to.
- ``serviceClassName`` comes from the file name where the service definition exists. 
- ``handler`` is the name of the function or method that handles the incoming service message. This function is called each time the service is called, and the message from the service call is passed to the handler as an argument. The handler should return an appropriate service response message.

### Using Services
Define a ServiceProxy, which provides the interface for sending messages to the service:
```python
service_proxy = rospy.ServiceProxy('service_name', serviceClassName)
```
One way the ServiceProxy can then be used to send requests is as follows:
```python
msg = serviceClassNameRequest()
#update msg attributes here to have correct data
response = service_proxy(msg)
```

### Creating a new service definition

```bash
cd ~/catkin_ws/src/simple_arm/
mkdir srv
cd srv
touch GoToPosition.srv
```

edit GoToPosition.srv
```
float64 joint_1
float64 joint_2
---
duration time_elapsed
```

The first section is the definition of the request message. 

The second section contains is the service response.

The time_elapsed field is of type duration, and is responsible for indicating how long it took the arm to perform the movement.

### Modifying CMakeLists.txt

First, ensure that the ``find_package()`` macro lists ``std_msgs`` and ``message_generation`` as required packages.

```
find_package(catkin REQUIRED COMPONENTS
        std_msgs
        message_generation
)
```

- ``std_msgs`` package contains all of the basic message types
- ``message_generation`` is required to generate message libraries
