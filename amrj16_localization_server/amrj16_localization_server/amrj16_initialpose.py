import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

class InitializePose(Node):
    def __init__(self):
        super().__init__('initialpose_node')
        
        # Initialize PoseWithCovarianceStamped and Odometry messages
        self._posedata = PoseWithCovarianceStamped()
        self._odomdata = Odometry()

        # Publisher for the 'amrj16/initialpose' topic
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Subscriber to the '/odom' topic
        self._odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Timer to periodically execute the timer_callback function
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.flag = True

    def timer_callback(self):
        # Call set_pose in the timer callback to publish the pose data
        self.set_pose()

    def odom_callback(self, msg):
        self._odomdata = msg  # Store the received odometry data

    def set_pose(self): 
        while rclpy.ok():
            connections = self._pose_pub.get_subscription_count()
            if connections > 0 and self.flag:
                self._posedata.header.frame_id = "map"
                self._posedata.header.stamp = self.get_clock().now().to_msg()

                # Fill in pose data from odom
                self._posedata.pose.pose.position.x = self._odomdata.pose.pose.position.x
                self._posedata.pose.pose.position.y = self._odomdata.pose.pose.position.y
                self._posedata.pose.pose.position.z = self._odomdata.pose.pose.position.z
                self._posedata.pose.pose.orientation.x = self._odomdata.pose.pose.orientation.x
                self._posedata.pose.pose.orientation.y = self._odomdata.pose.pose.orientation.y
                self._posedata.pose.pose.orientation.z = self._odomdata.pose.pose.orientation.z
                self._posedata.pose.pose.orientation.w = self._odomdata.pose.pose.orientation.w

                # Publish the pose
                self._pose_pub.publish(self._posedata)
                self.get_logger().warn(f"Publishing to /initialpose with {connections} connection(s)...")

                self.flag = False  # Prevent further publishing
            else:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = InitializePose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()