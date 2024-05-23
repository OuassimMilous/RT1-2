#!/usr/bin/env python
"""
Node_A UI

This node waits for user input to either change the robot's target or cancel the current action.

.. module:: Node_A_UI
    :platform: Unix
    :synopsis: User interface node for controlling the robot's target.

.. moduleauthor:: Ouassim Milous

Subscribes to:
    /odom

Publishes to:
    /posvelo
    /last_target

Client:
    /reaching_goal

Services:
    None
"""

from __future__ import print_function
import sys    
import rospy
import actionlib
from ouass.msg import PlanningAction, PlanningGoal
from ouass.msg import Data
from ouass.msg import RobotTarget
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def cancel():
    """
    Cancel the current goal.

    This function cancels the active goal.
    """
    client.cancel_goal()
    rospy.loginfo("Goal has been canceled\n")

def change_target():
    """
    Change the robot's target.

    This function reads coordinates from the user and sets them as the new target.
    """
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

def subscriber_callback(data):
    """
    Callback for the subscriber.

    This function extracts the X and Y data from the odometry message and republishes them reformatted.

    Args:
        data (Odometry): The odometry message containing both X and Y axis position and velocity.
    """

    # Declaring the custom message
    msg = Data()
    # Getting the current positions and velocities
    msg.vel_x = data.twist.twist.linear.x 
    msg.vel_y = data.twist.twist.linear.y
    msg.position_x = data.pose.pose.position.x 
    msg.position_y = data.pose.pose.position.y 

    # Declaring the publisher and the topic
    pub = rospy.Publisher("/posvelo", Data, queue_size=10)
    """ instance of the Publisher that publishes the refromatted X and Y axis position and velocity.
    """
    pub2 = rospy.Publisher('last_target', RobotTarget, queue_size=10)
    """ instance of the Publisher that publishes the current target.
    """
    # Publishing the message
    pub.publish(msg)

def main():
    """
    Main function.

    This function contains the main loop for the system, listening to user input to initiate changing or canceling the target.
    """
    while True:
        # Getting user input
        user_input = input("please enter 1 to change the target or 2 to cancel the current one:  ")
        # Execute the relevant function based on user input
        if user_input == "1":
            change_target()
        elif user_input == "2":
            cancel()

# Node initialization
if __name__ == '__main__':
    rospy.init_node('UI')
    # Client initialization and setting up the server
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    sub = rospy.Subscriber("/odom", Odometry, subscriber_callback)
    main()
    rospy.spin()
