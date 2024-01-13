#!/usr/bin/env python

import rospy
from math import hypot

from ouass.msg import Data
from ouass.msg import RobotTarget
from ouass.srv import TargetDistance, TargetDistanceResponse


class DistanceToTargetServer:
    def __init__(self, name: str) -> None:
        rospy.init_node(name, anonymous=False)  
        self.robot_speed_list = []                                                     
        self.av_speed_window_limit = rospy.get_param('av_window_size', default=10)      
        self.robot_current = Data()                                               
        self.target_position = RobotTarget()                                            
        self.is_target_set = False      

        rospy.Subscriber("/last_target", RobotTarget, self.robot_target_callback)     
        rospy.Subscriber("/posvelo", Data, self.robot_state_callback)
        self.service = rospy.Service("get_target_distance", TargetDistance, self.handle_get_target_distance)
                                      

    def robot_target_callback(self, data):
        self.target_position.target_x = data.target_x
        self.target_position.target_y = data.target_y
        if not self.is_target_set:

            self.is_target_set = True

    def robot_state_callback(self, data):
        self.robot_current.position_x = data.position_x
        self.robot_current.position_y = data.position_y
        if len(self.robot_speed_list) < self.av_speed_window_limit:
            self.robot_speed_list.append((data.vel_x, data.vel_y))
        elif len(self.robot_speed_list) == self.av_speed_window_limit:
            self.robot_speed_list.pop(0)
            self.robot_speed_list.append((data.vel_x, data.vel_y))

    def handle_get_target_distance(self, req):

        response = TargetDistanceResponse()
        if self.is_target_set:
            # Calculate distance
            response.dist_x = self.target_position.target_x - self.robot_current.position_x
            response.dist_y = self.target_position.target_y - self.robot_current.position_y
            rospy.loginfo("target_x = %d target_y %d", self.target_position.target_x, self.target_position.target_y)
            # distance as a line
            response.dist = hypot((self.target_position.target_x - self.robot_current.position_x), \
                                  (self.target_position.target_y - self.robot_current.position_y))
        else:
            # There is no target, set distance to zero
            response.dist_x = 0.0
            response.dist_y = 0.0
        # calculate average speed
        response.av_speed_x = sum( x[0] for x in self.robot_speed_list) / len(self.robot_speed_list)
        response.av_speed_y = sum( y[1] for y in self.robot_speed_list) / len(self.robot_speed_list)
        return response


def main():
    last_target_server_node = DistanceToTargetServer('get_target_distance')
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
