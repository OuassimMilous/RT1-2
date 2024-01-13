
# Second Assignment for Research Track I - ROS Robot Control and Monitoring

This repository contains ROS (Robot Operating System) nodes and services for controlling and monitoring a robot's movements. The system allows users to set targets for the robot, cancel ongoing actions, and obtain information about the robot's current state, target positions, distances, and average speed.

## Prerequisites

- ROS installed on your system ([ROS Installation Instructions](http://wiki.ros.org/ROS/Installation))
- Catkin workspace configured

## Getting Started

1. Clone the repository into your catkin workspace:
 ```
 git clone https://github.com/OuassimMilous/RT1-2
```
Build the workspace:
   ```
  cd catkin_ws
  catkin_make1
```

Source the workspace:
   ```
   source devel/setup.bash
```

Launch the necessary ROS nodes:
```
roslaunch robot_control robot_control.launch
```

## Nodes and Services
1. node_A.py
Listens for user input to either change the robot's target or cancel the current action.
Subscribes to the /odom topic to monitor the robot's odometry.
Connects to the /reaching_goal action server to send goals.
2. last_target_service_node.py
Implements a service server to provide the last set target coordinates.
Subscribes to the /last_target topic to update the last set target.
3. distance_to_target_server.py
Implements a service server to calculate the distance and average speed to the last set target.
Subscribes to the /last_target and /posvelo topics for robot and target information.

Usage

Run the node_A.py script to control the robot:
```rosrun robot_control node_A.py```

Access the last set target coordinates:
```rosservice call /get_last_target```

Access the distance of the target from the robot as well as the average velocity:
```rosservice call /get_last_target```

## The Pseudo codes

nodeA.py:
```
# Import necessary libraries and modules
import rospy
from actionlib import SimpleActionClient
from custom_msgs.msg import CustomMessage
from geometry_msgs.msg import Point
from std_msgs.msg import String

# Define a function to cancel the target
function cancel():
    cancel the goal in the action client
    log information that the goal has been canceled

# Define a function to change the robot target
function change_target():
    get user input for x and y coordinates
    create and publish a message for the last target
    wait for the action server to be available
    initialize a goal with the given x and y coordinates
    send the goal to the action server

# Define a callback function for the subscriber
function subscriber_callback(data):
    create a custom message
    extract current positions and velocities from the received data
    create a publisher for the "/posvelo" topic
    publish the custom message

# Define the main function
function main():
    continuously loop:
        get user input
        if input is "1":
            call change_target() function
        elif input is "2":
            call cancel() function

# Initialize the ROS node
if __name__ == '__main__':
    initialize the ROS node with the name 'node_A'
    initialize the action client for reaching_goal
    create a subscriber for the "/odom" topic with the subscriber_callback function
    call the main() function
    spin the ROS node



```
nodeB.py:

```
# Import necessary libraries and modules

# Define a class LastTargetServiceNode
class LastTargetServiceNode:
    # Constructor
    def __init__(self):
        initialize the ROS node with the name 'last_target_service_node'
        set self.last_target to None
        create a subscriber for the "/last_target" topic with the robot_target_callback function
        create a ROS service 'get_last_target' with LastTargetRequest and LastTargetResponse types
        set the service callback function to handle_get_last_target
        log information "Service LastTarget is ready"

    # Service callback function
    def handle_get_last_target(self, request):
        create a response variable of type LastTargetResponse
        if self.last_target is not None:
            set response.target_x to self.last_target[0]
            set response.target_y to self.last_target[1]
            log information "Received Robot Target:"
            log information f"  Target X: {response.target_x}"
            log information f"  Target Y: {response.target_y}"
        else:
            log warning "No target found"
        return response

    # Callback function
    def robot_target_callback(self, data):
        set self.last_target to (data.target_x, data.target_y)

# Main block
if __name__ == '__main__':
    try:
        instantiate an object of the LastTargetServiceNode class
        spin the ROS node

    except rospy.ROSInterruptException:
        pass


```

nodeC.py
```
# Import necessary libraries
import rospy
from math import hypot
from ouass.msg import Data
from ouass.msg import RobotTarget
from ouass.srv import TargetDistance, TargetDistanceResponse

# Define the DistanceToTargetServer class
class DistanceToTargetServer:
    # Constructor
    def __init__(self, name: str) -> None:
        # Initialize the ROS node
        rospy.init_node(name, anonymous=False)  

        # Initialize variables
        self.robot_speed_list = []                                                     
        self.av_speed_window_limit = rospy.get_param('av_window_size', default=10)      
        self.robot_current = Data()                                               
        self.target_position = RobotTarget()                                            
        self.is_target_set = False      

        # Subscribe to topics for target and robot state
        rospy.Subscriber("/last_target", RobotTarget, self.robot_target_callback)     
        rospy.Subscriber("/posvelo", Data, self.robot_state_callback)

        # Create a service named 'get_target_distance' with the TargetDistance service type
        self.service = rospy.Service("get_target_distance", TargetDistance, self.handle_get_target_distance)

    # Callback function for handling RobotTarget messages
    def robot_target_callback(self, data):
        self.target_position.target_x = data.target_x
        self.target_position.target_y = data.target_y
        if not self.is_target_set:
            self.is_target_set = True

    # Callback function for handling Data messages
    def robot_state_callback(self, data):
        self.robot_current.position_x = data.position_x
        self.robot_current.position_y = data.position_y
        if len(self.robot_speed_list) < self.av_speed_window_limit:
            self.robot_speed_list.append((data.vel_x, data.vel_y))
        elif len(self.robot_speed_list) == self.av_speed_window_limit:
            self.robot_speed_list.pop(0)
            self.robot_speed_list.append((data.vel_x, data.vel_y))

    # Service handler function
    def handle_get_target_distance(self, req):
        response = TargetDistanceResponse()

        if self.is_target_set:
            response.dist_x = self.target_position.target_x - self.robot_current.position_x
            response.dist_y = self.target_position.target_y - self.robot_current.position_y
            response.dist = hypot((self.target_position.target_x - self.robot_current.position_x),
                                  (self.target_position.target_y - self.robot_current.position_y))
            rospy.loginfo("target_x = %d target_y %d", self.target_position.target_x, self.target_position.target_y)
        else:
            response.dist_x = 0.0
            response.dist_y = 0.0

        response.av_speed_x = sum(x[0] for x in self.robot_speed_list) / len(self.robot_speed_list)
        response.av_speed_y = sum(y[1] for y in self.robot_speed_list) / len(self.robot_speed_list)

        return response

# Main function
def main():
    last_target_server_node = DistanceToTargetServer('get_target_distance')
    rospy.spin()

# Entry point
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


```
