import math
import os
from datetime import datetime
 
import cv2
from cv_bridge import CvBridge
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
 
 
class ObstacleAvoid(Node):
 
    FORWARD = 'FORWARD'                    # Driving straight ahead
    STOP_AND_CAPTURE = 'STOP_AND_CAPTURE'  # Stopped, waiting for the photo to be saved
    TURNING = 'TURNING'                    # Steering away from the obstacle in an arc
 
    def __init__(self):
        super().__init__('obstacle_avoid')
 
        self.get_logger().info('obstacle_avoid node started')
 
        # ── Movement parameters ───────────────────────────────────────────────
 
        # If anything is detected closer than 0.8 m straight ahead, the robot stops.
        self.safe_distance = 0.8
 
        #driving speed in metres per second of the robot
        self.forward_speed = 0.2
 
        # How sharply the robot steers during the avoidance arc, in radians per second.
        # A higher number means a tighter curve.
        self.turn_rate = 0.5
 
        # The robot must rotate this far before it goes straight again.
        self.target_turn_angle = math.radians(45.0)
 
        # ── LiDAR cone settings ───────────────────────────────────────────────
 
        # We only care about laser beams that point within ±30° of straight ahead.
        self.front_cone_half_angle = math.radians(30.0)
 
        # Readings shorter than 15 cm are almost certainly sensor noise — throw them away.
        self.min_valid_distance = 0.15
 
        # ── State variables ───────────────────────────────────────────────────
 
        # The robot always starts by moving forward.
        self.state = self.FORWARD
 
        # store the most recent LiDAR message here so the timer can use it.
        self.last_scan = None
 
        # The robot's current heading angle in radians, updated from /odom.
        # 0 = facing along the X axis; positive = rotated counter-clockwise.
        self.current_yaw = 0.0
 
        # We snapshot the heading when a turn begins so we can measure progress.
        self.turn_start_yaw = None
 
        # 1.0 = steer left (counter-clockwise). Set to -1.0 to always steer right.
        self.turn_direction = 1.0
 
        # ── Photo handshake flags ─────────────────────────────────────────────
 
        # Setting capture_requested = True tells image_callback to save the very
        # next camera frame it receives.
        self.capture_requested = False
 
        # image_callback sets this to True once the file is on disk.
        # The timer reads this flag to know it is safe to start turning.
        self.image_saved = False
 
        # ── Camera setup ──────────────────────────────────────────────────────
 
        # CvBridge converts ROS image messages into OpenCV arrays we can save.
        self.bridge = CvBridge()
 
        # All obstacle photos are saved here. The folder is created automatically
        # if it does not already exist.
        self.image_dir = '/mnt/c/Users/HP/Documents/image_captured'
        os.makedirs(self.image_dir, exist_ok=True)
 
        # ── ROS communication ─────────────────────────────────────────────────
 
        # We publish velocity commands to this topic to physically move the robot.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
 
        # Receive laser scan data so we know when an obstacle is approaching.
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
 
        # Receive camera frames so we can photograph the obstacle.
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data
        )
 
        # Receive odometry so we can measure exactly how far the robot has turned.
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
 
        # This timer fires 10 times per second
        # Every tick it checks the current state and decides what the robot should do next.
        self.timer = self.create_timer(0.1, self.timer_callback)
 
    # ── Sensor callbacks ──────────────────────────────────────────────────────
 
    def scan_callback(self, msg):
        # Store the latest laser scan so the timer can read it on the next tick.
        self.last_scan = msg
 
    def odom_callback(self, msg):
        # The robot's orientation arrives as a quaternion (four numbers: x, y, z, w).
        # We only need the yaw (left/right rotation), so we extract it using the
        # standard quaternion-to-yaw formula. The result is in radians.
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
 
    def image_callback(self, msg):
        # This callback fires every time a new camera frame arrives, which could be
        # 30 times per second. We only want ONE photo per obstacle, so we use the
        # capture_requested flag as a one-shot trigger — once it fires we clear it
        # immediately so all future frames are ignored until the next obstacle.
        if not self.capture_requested:
            return
 
        # Convert the ROS image message into an OpenCV image (BGR colour format).
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
 
        # Build a unique filename using the current timestamp so no two photos
        # ever overwrite each other, even if the robot runs for a long time.
        filename = os.path.join(
            self.image_dir,
            f'obstacle_{datetime.now().strftime("%Y%m%d_%H%M%S")}.png',
        )
 
        # Write the image file to disk.
        cv2.imwrite(filename, cv_image)
 
        # Lower the trigger flag and raise the "done" flag so the timer knows
        # the photo is safely saved and the robot can begin turning.
        self.capture_requested = False
        self.image_saved = True
        self.get_logger().info(f'Image saved: {filename}')
 
    # ── Helper functions ──────────────────────────────────────────────────────
 
    def front_min_distance(self):
        # Go through every laser reading and collect only the ones that point
        # within our ±30° front cone and are physically plausible (> 15 cm).
        # Return the shortest of those readings, or None if there are no valid ones.
        if self.last_scan is None:
            return None
 
        values = []
        for i, r in enumerate(self.last_scan.ranges):
            angle = self.last_scan.angle_min + i * self.last_scan.angle_increment
            if abs(angle) <= self.front_cone_half_angle:
                if math.isfinite(r) and r > self.min_valid_distance:
                    values.append(r)
 
        return min(values) if values else None
 
    def _angle_diff(self, a, b):
        # Calculate the shortest angular gap between two headings.
        # Without this wrap-around logic, crossing the ±180° boundary would give
        # a wrong result — e.g. going from 170° to -170° would look like 340°
        # instead of the correct 20°. The result is always in [-180°, +180°].
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d
 
    # ── Main decision loop (runs 10 times per second) ─────────────────────────
 
    def timer_callback(self):
        # A Twist message with all zeros means "stop". We build one here and
        # each branch below either modifies it or publishes it as-is.
        cmd = Twist()
 
        # Safety check: wait until we have at least one laser scan before moving.
        if self.last_scan is None:
            self.cmd_pub.publish(cmd)
            return
        # ── Stage 1 — FORWARD ────────────────────────────────────────────────
        # Normal driving. Every tick we measure the closest object ahead.
        if self.state == self.FORWARD:
            front = self.front_min_distance()
 
            if front is not None and front < self.safe_distance:
                # Obstacle too close — stop the robot and request a photo.
                self.capture_requested = True
                self.image_saved = False
                self.state = self.STOP_AND_CAPTURE
                self.get_logger().info(
                    f'Obstacle at {front:.2f} m — stopping and capturing image'
                )
                self.cmd_pub.publish(cmd)   # zero velocity = full stop
                return
            # Nothing too close — keep driving straight ahead.
            cmd.linear.x = self.forward_speed
            self.cmd_pub.publish(cmd)
            return
        # ── Stage 2 — STOP AND CAPTURE ───────────────────────────────────────
        #image_callback confirms the photo is on disk
        if self.state == self.STOP_AND_CAPTURE:
            self.cmd_pub.publish(cmd)   # stay stopped
 
            if self.image_saved:
                # Photo confirmed on disk — snapshot the current heading so we
                # can measure the turn angle from this exact point.
                self.turn_start_yaw = self.current_yaw
                self.state = self.TURNING
                self.get_logger().info('Image saved — starting 45° arc turn')
            return
 
        # ── Stage 3 — TURNING ────────────────────────────────────────────────
        # The robot steers away from the obstacle by driving an arc.
        # spin in place — it must always have forward speed in order to steer.
        # So we publish both linear.x (forward) and angular.z (steer) together.
 
        if self.state == self.TURNING:
            # How many radians have we turned since the turn began?
            turned = abs(self._angle_diff(self.current_yaw, self.turn_start_yaw))
 
            if turned >= self.target_turn_angle:
                # 45 degrees reached — straighten the steering and go back to FORWARD.
                self.get_logger().info('45° turn complete — moving forward')
                self.turn_start_yaw = None
                self.state = self.FORWARD
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return
 
            # Still mid-turn — drive forward and steer at the same time.
            # linear.x moves the robot forward; angular.z turns the wheels.
            cmd.linear.x = self.forward_speed
            cmd.angular.z = self.turn_direction * self.turn_rate
            self.cmd_pub.publish(cmd)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
 
 
 
 