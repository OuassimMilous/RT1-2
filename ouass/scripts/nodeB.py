#!/usr/bin/env python

import rospy
from ouass.srv import LastTarget, LastTargetResponse
from ouass.msg import RobotTarget

class LastTargetServiceNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('last_target_service_node')

        # Initialize the last_target variable
        self.last_target = None

        # Subscribe to the "/last_target" topic to get the latest robot target coordinates
        rospy.Subscriber("/last_target", RobotTarget, self.robot_target_callback)

        # Create a ROS service named 'get_last_target' with the LastTarget service type
        self.service = rospy.Service('get_last_target', LastTarget, self.handle_get_last_target)

        # Log information that the service is ready
        rospy.loginfo("Service LastTarget is ready")

    def handle_get_last_target(self, request):
        # Handle service requests and prepare the response
        response = LastTargetResponse()

        if self.last_target:
            # If there is a last_target, fill in the response with its coordinates
            response.target_x = self.last_target[0]
            response.target_y = self.last_target[1]
            rospy.loginfo("Received Robot Target:")
            rospy.loginfo(f"  Target X: {response.target_x}")
            rospy.loginfo(f"  Target Y: {response.target_y}")
        else:
            # If there is no last_target, log a warning
            rospy.logwarn("No target found")

        # Return the response to the service client
        return response

    def robot_target_callback(self, data):
        # Callback function to update last_target when a new RobotTarget message is received
        self.last_target = (data.target_x, data.target_y)

if __name__ == '__main__':
    try:
        # Instantiate the LastTargetServiceNode class
        last_target_service_node = LastTargetServiceNode()

        # Spin to keep the script running
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
