
import time

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/line_follower/camera/image',
            self.camera_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        
        self.base_speed = 0.1
        self.max_angular = 0.4
        self.min_dark_pixels = 10

        # PID gains
        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.0

        
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

        # Line-loss recovery
        self.last_turn_direction = 1
        self.line_lost_since = None
        self.search_duration = 30

        self.get_logger().info(
            'PID line follower started'
        )

    def reset_pid(self):
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

    def camera_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as error:
            self.get_logger().error(
                f'Image conversion failed: {error}'
            )
            return

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        threshold, mask = cv2.threshold(
            gray,
            0,
            255,
            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        height, width = mask.shape

        # Inspect the bottom 30% of the image
        roi = mask[int(height * 0.70):, :]

        dark_pixels = cv2.countNonZero(roi)
        now = time.monotonic()

        command = Twist()

        
        if dark_pixels >= self.min_dark_pixels:
            moments = cv2.moments(roi)

            if moments['m00'] <= 0:
                self.handle_line_loss(command, now)
                self.cmd_pub.publish(command)
                return

           
            center_x = (
                moments['m10'] / moments['m00']
            )

            
            error = center_x - width / 2.0

            
            if self.previous_time is None:
                dt = 0.0
                derivative = 0.0
            else:
                dt = now - self.previous_time

                if dt > 0 and self.previous_error is not None:
                    derivative = (
                        error - self.previous_error
                    ) / dt
                else:
                    derivative = 0.0

            
            if dt > 0:
                self.integral += error * dt

            
            self.integral = max(
                -1000.0,
                min(1000.0, self.integral)
            )

            
            p_term = self.kp * error
            i_term = self.ki * self.integral
            d_term = self.kd * derivative

            correction = p_term + i_term + d_term

            
            angular = -correction

            angular = max(
                -self.max_angular,
                min(self.max_angular, angular)
            )

            
            turn_fraction = min(
                abs(angular) / self.max_angular,
                1.0
            )

            command.linear.x = (
                self.base_speed
                * (1.0 - 0.5 * turn_fraction)
            )

            command.angular.z = angular

            if angular < -0.01:
                self.last_turn_direction = -1
            elif angular > 0.01:
                self.last_turn_direction = 1

            
            self.previous_error = error
            self.previous_time = now

            
            self.line_lost_since = None

            self.get_logger().info(
                f'Pixels: {dark_pixels}, '
                f'Center: {center_x:.1f}, '
                f'Error: {error:.1f}, '
                f'P: {p_term:.3f}, '
                f'I: {i_term:.3f}, '
                f'D: {d_term:.3f}, '
                f'Angular: {angular:.3f}',
                throttle_duration_sec=0.5
            )
        else:
            self.reset_pid()
            self.handle_line_loss(command, now)

        self.cmd_pub.publish(command)

    def handle_line_loss(self, command, now):
        if self.line_lost_since is None:
            self.line_lost_since = now

        lost_time = now - self.line_lost_since
        
        direction = self.last_turn_direction
        command.linear.x = 0.0
        command.angular.z = direction * 0.15

        self.get_logger().warn(
            f'Line lost: {lost_time:.1f}s; '
            f'search direction={direction}',
            throttle_duration_sec=1.0
        )

    def destroy_node(self):
        self.cmd_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
