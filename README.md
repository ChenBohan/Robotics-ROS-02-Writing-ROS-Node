# Robotics-ROS-01-Writing-ROS-Node
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
  - Synchronous publishing means that a publisher will attempt to publish to a topic but may be blocked if that topic is being published to by a different publisher. In this situation, the second publisher is blocked until the first publisher has serialized all messages to a buffer and the buffer has written the messages to each of the topic's subscribers. This is the default behavior of a rospy.Publisher if the queue_size parameter is not used or set to None.
  - Asynchronous publishing means that a publisher can store messages in a queue until the messages can be sent. If the number of messages published exceeds the size of the queue, the oldest messages are dropped. The queue size can be set using the queue_size parameter.
  
## Simple Mover (Example)

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

