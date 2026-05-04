import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # message type for robot velocity
from sensor_msgs.msg import LaserScan  # message type for LiDAR data
 
 
# Define a class called Print that inherits from Node
class Print(Node):
 
    def __init__(self):
        # Initialize the node with name "Print"
        super().__init__('Print')
 
        # Initialize variable to store velocity values
        self.linear = 0.0
        self.angular = 0.0
 
        # Variable to store LiDAR readings
        self.scan_values = []
 
        # Subscribe to /cmd_vel topic to get robot velocity
        self.vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.vel_callback, 10
        )
 
        # Subscribe to /scan topic to get LiDAR data
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
 
        # Create a timer that runs every 5 seconds
        self.timer = self.create_timer(5.0, self.timer_callback)
 
    # Callback function for velocity message
    def vel_callback(self, msg):
        # Store linear velocity (forward/backward)
        self.linear = msg.linear.x
 
        # Store angular velocity (rotation)
        self.angular = msg.angular.z
 
    # Callback function for LiDAR message
    def scan_callback(self, msg):
        # Store all LiDAR distance readings
        self.scan_values = list(msg.ranges)
 
        # Get total number of readings
        n = len(self.scan_values)
 
        # Find the center index (front of robot)
        self.center = n // 2
 
        # Define a small window around the center (+15, -15 values)
        self.left = self.center - 15
        self.right = self.center + 15
 
    # Timer function that runs every 5 seconds
    def timer_callback(self):
        if len(self.scan_values) > 0:
 
            # Get front LiDAR values
            front_values = self.scan_values[self.left:self.right + 1]
 
            # Get minimum distance in front
            min_distance = min(front_values)
 
 
            self.get_logger().info('-----------------------------')
            # Print the output
            self.get_logger().info(
                f'Forward Velocity: {self.linear:.2f} m/s'
            )
            self.get_logger().info(
                f'Turning Rate: {self.angular:.2f} rad/s'
            )
            self.get_logger().info(
                f'Front Distance: {min_distance:.2f} m'
            )
            self.get_logger().info('-----------------------------')
 
 
# Main function to run the node
def main(args=None):
    # Initialize ROS2 communication
    rclpy.init(args=args)
 
    # Create node object
    node = Print()
 
    # Keep the node running
    rclpy.spin(node)
 
    # Shutdown when finished
    rclpy.shutdown()
 
 
# Run main function if file is executed
if __name__ == '__main__':
    main()