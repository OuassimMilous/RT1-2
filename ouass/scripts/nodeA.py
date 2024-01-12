#!/usr/bin/env python
from __future__ import print_function
import sys    
import rospy
import actionlib
from assignment_2_2023.msg import PlanningAction, PlanningGoal
from ouass.msg import Data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# a function to cancel the target
def cancel():
    client.cancel_goal()
    rospy.loginfo("Goal has been canceled\n")

# a function to take the robot to the target
def change_target():
    # getting the input for x and y
    x = float(input("Enter the x coordinates: "))
    y = float(input("Enter the y coordinates: "))    
    print(f'the new target coordinates: \n x: {x} \n y: {y}')    
    # wait for the server
    client.wait_for_server()  
    # initializing the goal  
    goal = PoseStamped()    
    goal.pose.position.x = x
    goal.pose.position.y = y
    # setting the goal
    goal = PlanningGoal(goal) 
    #sending the goal to the action server    
    client.send_goal(goal)
    
    
# a function for the subscriber callback
def subscriber_callback(data):
    # declaring the custom message
    msg = Data()
    # getting the current positions and velocities
    msg.vel_x = data.twist.twist.linear.x 
    msg.vel_y = data.twist.twist.linear.y
    msg.position_x = data.pose.pose.position.x 
    msg.position_y = data.pose.pose.position.y 

    # declaring the publisher and the topic
    pub = rospy.Publisher("posvelo", Data, queue_size=10)
    # publishing the msg
    pub.publish(msg)


def main():
    while True:
		# getting input
        user_input= input("user input:  ")
		# execute the relivent function
        if (user_input == "1"):
            change_target()
        elif (user_input == "2"):
            cancel()


if __name__ == '__main__':
	# node initialization
    rospy.init_node('node_A')
    # client initialization and setting up the server
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    sub = rospy.Subscriber("/odom", Odometry, subscriber_callback)
    main()
    rospy.spin()
    
    
    

