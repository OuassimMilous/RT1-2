#!/usr/bin/env python

import rospy
from ouass.srv import LastTarget, LastTargetResponse
from ouass.msg import RobotTarget

class LastTargetServiceNode:
    def __init__(self):
        rospy.init_node('last_target_service_node')
        self.last_target = None
        rospy.Subscriber("/last_target", RobotTarget, self.robot_target_callback)
        self.service = rospy.Service('get_last_target', LastTarget, self.handle_get_last_target)
        rospy.loginfo("Service LastTarget is ready")

    def handle_get_last_target(self, request):
        response = LastTargetResponse()
        if self.last_target:
            response.target_x = self.last_target[0]
            response.target_y = self.last_target[1]
            print("Received Robot Target:")
            print("  Target X:", response.target_x)
            print("  Target Y:", response.target_y)
        else:
            rospy.logwarn("No target found")
        return response

    def robot_target_callback(self, data):
        self.last_target = (data.target_x, data.target_y)

if __name__ == '__main__':
    try:
        last_target_service_node = LastTargetServiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

