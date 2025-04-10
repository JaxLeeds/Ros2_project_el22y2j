import threading
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from math import sin, cos, atan2
import signal


class Robot(Node):
    def __init__(self):
        super().__init__('robot')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.colour_flags = {'red': False, 'green': False, 'blue': False}
        self.sensitivity = 10
        self.see_all_colours = False
        self.blue_contour_area = 0
        self.goal_sent = False

        self.forward_threshold = 5000
        self.back_threshold = 15000
        self.seen_blue_contour = None

    def callback(self, data):
        try:
            #transform ROS image message into OpenCV graph
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'transform fail: {e}')
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # identify HSV colour range
        masks = {
            'red': cv2.inRange(hsv, (0, 100, 100), (10, 255, 255)) | cv2.inRange(hsv, (160, 100, 100), (180, 255, 255)),
            'green': cv2.inRange(hsv, (60 - self.sensitivity, 100, 100), (60 + self.sensitivity, 255, 255)),
            'blue': cv2.inRange(hsv, (110 - self.sensitivity, 100, 100), (110 + self.sensitivity, 255, 255)),
        }

        # Reset colour flag
        for key in self.colour_flags:
            self.colour_flags[key] = False

        # 遍历每种颜色，检测轮廓并绘制
        for colour, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > 500:
                    M = cv2.moments(c)
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])

                        cv2.drawContours(image, [c], -1, (255, 255, 255), 2)
                        cv2.circle(image, (cx, cy), 5, (0, 0, 0), -1)
                        cv2.putText(image, colour, (cx - 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                        self.colour_flags[colour] = True

                        if colour == 'blue':
                            self.blue_contour_area = area
                            self.seen_blue_contour = (cx, cy)

        # if three colours can be detected
        self.see_all_colours = all(self.colour_flags.values())

        # 显示图像
        cv2.namedWindow('camera_Feed', cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed', 320, 240)
        cv2.waitKey(3)

    def walk_forward(self):
        msg = Twist()
        msg.linear.x = 0.15
        self.publisher.publish(msg)

    def walk_backward(self):
        msg = Twist()
        msg.linear.x = -0.15
        self.publisher.publish(msg)

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.5
        self.publisher.publish(msg)

    def turn_right(self):
        msg = Twist()
        msg.angular.z = -0.5
        self.publisher.publish(msg)

    def stop(self):
        msg = Twist()
        self.publisher.publish(msg)

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal_msg)


def main():
    rclpy.init()
    robot = Robot()

    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()
        cv2.destroyAllWindows()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    # 蓝色方块的地图坐标（假设静态），你可根据实际Gazebo情况调整
    blue_box_pos = (1.5, 5.0)
    robot_start = (1.0, 1.0)

    try:
        while rclpy.ok():
            if robot.see_all_colours:
                if robot.colour_flags['blue']:
                    # 如果蓝色靠近并未导航，导航过去
                    if robot.blue_contour_area > 8000 and not robot.goal_sent:
                        dx = blue_box_pos[0] - robot_start[0]
                        dy = blue_box_pos[1] - robot_start[1]
                        dist = (dx**2 + dy**2)**0.5
                        ratio = max(0.0, dist - 1.0) / dist
                        goal_x = robot_start[0] + dx * ratio
                        goal_y = robot_start[1] + dy * ratio
                        yaw = atan2(dy, dx)

                        robot.get_logger().info("detect the blue block, start navigating...")
                        robot.send_goal(goal_x, goal_y, yaw)
                        robot.goal_sent = True
                    else:
                        robot.stop()
                else:
                    # 绿方块调节远近（可选逻辑）
                    if robot.blue_contour_area > robot.back_threshold:
                        robot.walk_backward()
                    elif robot.blue_contour_area < robot.forward_threshold:
                        robot.walk_forward()
                    else:
                        robot.stop()
            else:
                # 未看到所有颜色：探索模式（原地左转）
                robot.turn_left()
            time.sleep(0.1)

    except:
        pass

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
