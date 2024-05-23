#!/usr/bin/env python
"""
Node_C DistanceToTargetServer

This node calculates the distance to the target and the robot's average speed.

.. module:: Node_C_DistanceToTargetServer
    :platform: Unix
    :synopsis: Calculates the distance to the target and the robot's average speed.

.. moduleauthor:: Ouassim Milous

Subscribes to:
    /last_target
    /posvelo

Publishes to:
    None

Services:
    /get_target_distance

Clients:
    None
"""

import rospy
from math import hypot

from ouass.msg import Data
from ouass.msg import RobotTarget
from ouass.srv import TargetDistance, TargetDistanceResponse

class DistanceToTargetServer:
    """
    A class to represent the Distance to Target Server.

    This node subscribes to the /last_target and /posvelo topics to get the target and robot state information, 
    and provides a service to calculate and return the distance to the target and the robot's average speed.
    """
    
    def __init__(self, name: str) -> None:
        """
        Initialize the DistanceToTargetServer.

        This method initializes the ROS node, sets up the subscribers, and the service server, 
        and initializes variables for robot state and target position.

        Args:
            name (str): The name of the ROS node.
        """
        # Initialize the ROS node
        rospy.init_node(name, anonymous=False)  

        # Variables to store robot speed, window limit, current robot state, target position, and target set status
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

    def robot_target_callback(self, data):
        """
        Callback function to update the target position when a new RobotTarget message is received.

        Args:
            data (RobotTarget): The RobotTarget message containing the target coordinates.
        """
        self.target_position.target_x = data.target_x
        self.target_position.target_y = data.target_y
        if not self.is_target_set:
            self.is_target_set = True

    def robot_state_callback(self, data):
        """
        Callback function to update the current robot state when a new Data message is received.

        Args:
            data (Data): The Data message containing the robot's current state.
        """
        self.robot_current.position_x = data.position_x
        self.robot_current.position_y = data.position_y
        if len(self.robot_speed_list) < self.av_speed_window_limit:
            self.robot_speed_list.append((data.vel_x, data.vel_y))
        elif len(self.robot_speed_list) == self.av_speed_window_limit:
            self.robot_speed_list.pop(0)
            self.robot_speed_list.append((data.vel_x, data.vel_y))

    def handle_get_target_distance(self, req):
        """
        Service handler function to calculate and return the distance and average speed.

        This method calculates the distance to the target and the robot's average speed, and prepares the response.

        Args:
            req: The service request.

        Returns:
            TargetDistanceResponse: The response containing the distance to the target and average speed.
        """
        # Prepare the response message
        response = TargetDistanceResponse()

        if self.is_target_set:
            # Calculate distance in x and y directions
            response.dist_x = self.target_position.target_x - self.robot_current.position_x
            response.dist_y = self.target_position.target_y - self.robot_current.position_y

            # Calculate total distance as a line
            response.dist = hypot((self.target_position.target_x - self.robot_current.position_x),
                                  (self.target_position.target_y - self.robot_current.position_y))
            rospy.loginfo("target_x = %d target_y %d", self.target_position.target_x, self.target_position.target_y)
        else:
            # If there is no target, set distance to zero
            response.dist_x = 0.0
            response.dist_y = 0.0

        # Calculate average speed in x and y directions
        response.av_speed_x = sum(x[0] for x in self.robot_speed_list) / len(self.robot_speed_list)
        response.av_speed_y = sum(y[1] for y in self.robot_speed_list) / len(self.robot_speed_list)

        # Return the response to the service client
        return response

def main():
    """
    Main function to instantiate the DistanceToTargetServer and keep the node running.
    """
    # Instantiate the DistanceToTargetServer class
    last_target_server_node = DistanceToTargetServer('get_target_distance')

    # Spin to keep the script running
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
