#!/usr/bin/env python
from __future__ import print_function
import sys    
import rospy
import actionlib
from ouass.msg import PlanningAction, PlanningGoal
from ouass.msg import Data
from ouass.msg import RobotTarget
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# Function to cancel the target
def cancel():
    client.cancel_goal()
    rospy.loginfo("Goal has been canceled\n")

# Function to take the robot to the target
def change_target():
    # Getting the input for x and y
    x = float(input("Enter the x coordinates: "))
    y = float(input("Enter the y coordinates: "))    
    print(f'The new target coordinates: \n x: {x} \n y: {y}')    
    # Publishing the last target for reference

    last_target_msg = RobotTarget()
    last_target_msg.target_x = x
    last_target_msg.target_y = y
    pub2 = rospy.Publisher('last_target', RobotTarget, queue_size=10)
    pub2.publish(last_target_msg)
    
    # Wait for the action server
    client.wait_for_server()  
    # Initializing the goal  
    goal = PoseStamped()    
    goal.pose.position.x = x
    goal.pose.position.y = y
    # Setting the goal
    goal = PlanningGoal(goal) 
    # Sending the goal to the action server    
    client.send_goal(goal)

# Callback for the subscriber
def subscriber_callback(data):
    # Declaring the custom message
    msg = Data()
    # Getting the current positions and velocities
    msg.vel_x = data.twist.twist.linear.x 
    msg.vel_y = data.twist.twist.linear.y
    msg.position_x = data.pose.pose.position.x 
    msg.position_y = data.pose.pose.position.y 

    # Declaring the publisher and the topic
    pub = rospy.Publisher("/posvelo", Data, queue_size=10)
    pub2 = rospy.Publisher('last_target', RobotTarget, queue_size=10)
    # Publishing the message
    pub.publish(msg)

# Main function
def main():
    while True:
        # Getting user input
        user_input = input("user input:  ")
        # Execute the relevant function based on user input
        if user_input == "1":
            change_target()
        elif user_input == "2":
            cancel()

# Node initialization
if __name__ == '__main__':
    rospy.init_node('node_A')
    # Client initialization and setting up the server
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    sub = rospy.Subscriber("/odom", Odometry, subscriber_callback)
    main()
    rospy.spin()
