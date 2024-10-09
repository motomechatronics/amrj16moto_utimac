#!/usr/bin/env python3

import rclpy
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from amrj16_messages.srv import PoiServiceMessage
import yaml


class SavePoi(Node):
    def __init__(self):
        super().__init__("save_poi_service_server_node")

        # Create the service server
        self.save_poi_service = self.create_service(
            PoiServiceMessage, "/save_poi", self.service_callback)
        
        self.get_logger().info("Launching server...") 
        self.get_logger().warn('Use a terminal to save the position with: ros2 service call /save_poi amrj16_messages/srv/PoiServiceMessage "{label: \'home\'}"')
        
        # Subscriber to the pose topic
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.subscriber_callback, 10)

        # Initialize position and orientation variables
        self.x = None
        self.y = None
        self.z = None
        self.a = None
        self.b = None
        self.g = None
        self.w = None
        
        self.dict = {}

        self.package_name = 'amrj16_localization_server'
        self.package_path = get_package_share_directory(self.package_name)

    def subscriber_callback(self, msg):
        # position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        # orientation
        self.a = msg.pose.pose.orientation.x
        self.b = msg.pose.pose.orientation.y
        self.g = msg.pose.pose.orientation.z
        self.w = msg.pose.pose.orientation.w

        self.get_logger().info(f"Received Pose: x={self.x}, y={self.y}, z={self.z}")

    def save_pose(self):
        self.get_logger().info("Data saved.")
        self.file_path = "/home/user/ros2_ws/src/amrj16_localization_server/config/poi.yaml"       
        with open(self.file_path , 'w') as file:
            yaml.dump(self.dict, file)

    def add_pose(self):
        if None in [self.x, self.y, self.z, self.a, self.b, self.g, self.w]:
            self.get_logger().info("Pose data is incomplete. Cannot add pose.")
        else:
            self.item = {self.data: {
                        'position': {
                            'x': self.x,
                            'y': self.y,
                            'z': self.z},
                        'orientation': {
                            'x': self.a,
                            'y': self.b,
                            'z': self.g,
                            'w': self.w}}}

            self.dict = {**self.item, **self.dict}
            self.get_logger().info(str(self.dict))
      
      

    def service_callback(self, request, response):
        self.data = request.label

        if self.data != "end":
            self.add_pose()
            response.output = "Add pose correctly"
        else:
            self.save_pose()
            response.output = "Pose(s) correctly saved and stop recording"
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SavePoi()
        rclpy.spin(node)
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
