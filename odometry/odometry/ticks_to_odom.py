import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int32
import math

def compute_velocity(ticks, prev_ticks, time_interval, wheel_radius, ppr):
    """Compute wheel velocity in m/s"""
    tick_diff = ticks - prev_ticks
    distance = (tick_diff / ppr) * (2 * math.pi * wheel_radius)
    return distance / time_interval

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('encoder_odometry')
        
        # Robot parameters
        self.wheel_radius = 0.0650  # in meters
        self.base_width = 0.362  # Distance between wheels in meters
        self.ppr = 560  # Encoder pulses per revolution
        
        # Initialize variables
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # ROS 2 publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Int32, 'left_ticks', self.left_encoder_callback, 10)
        self.create_subscription(Int32, 'right_ticks', self.right_encoder_callback, 10)

        
        # Timer for publishing odometry
        self.timer = self.create_timer(0.1, self.update_odometry)  # 10 Hz
        self.last_time = self.get_clock().now()
    
    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data
    
    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data
    
    def update_odometry(self):
        current_time = self.get_clock().now()
        time_interval = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        if time_interval == 0:
            return
        
        # Compute wheel velocities
        left_velocity = compute_velocity(self.left_ticks, self.prev_left_ticks, time_interval, self.wheel_radius, self.ppr)
        right_velocity = compute_velocity(self.right_ticks, self.prev_right_ticks, time_interval, self.wheel_radius, self.ppr)
        
        # Update previous ticks
        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks
        self.last_time = current_time
        
        # Compute robot velocity
        linear_velocity = (left_velocity + right_velocity) / 2.0
        angular_velocity = (right_velocity - left_velocity) / self.base_width
        
        # Update robot position
        delta_x = linear_velocity * math.cos(self.theta) * time_interval
        delta_y = linear_velocity * math.sin(self.theta) * time_interval
        delta_theta = angular_velocity * time_interval
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity
        self.odom_pub.publish(odom_msg)
        
        # Publish transform for tf2
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        transform.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(transform)
        
        self.get_logger().info(f'Odom: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

