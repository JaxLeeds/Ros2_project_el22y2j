#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from math import sin, cos

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        # Initialise a publisher to publish messages to the robot base 
        # We covered which topic receives messages that move the robot in the 3rd Lab Session
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz


#-----------------------------------Navigation-----------------------------------------------------------------------------------------
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navigation_done_event = threading.Event()
#-----------------------------------------------------------------------------------------------------------------------------

        # Initialise any flags that signal a colour has been detected (default to false)
        self.red_detected = False
        self.green_detected = False
        self.blue_detected = False


        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10


        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
        self.move_forwards_flag = False
        self.move_backwards_flag = False
        self.turn_left_flag =False
        self.turn_right_flag = False
        self.stop_flag = False
        
        
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use 
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning


        # We covered which topic to subscribe to should you wish to receive image data

#-----------------------------------------Navigation-------------------------------------------------------------------------------------------------------------------

    def send_next_goal(self, x, y, yaw):
        #if self.current_goal_index >= len(self.goals):
            #self.get_logger().info('all  goal points were navigated')
            #return

        #x, y, yaw = self.goals[self.current_goal_index]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)
        
        self.get_logger().info(f'sent goal point: ({x}, {y}, yaw={yaw})')
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)



    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.navigation_done_event.set()
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'goal point arrived')
        self.navigation_done_event.set()


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        #self.get_logger().info(f'distance to goal point: {distance:.2f}')
        
        #Access the current pose
        #current_pose = feedback_msg.feedback.current_pose
        #position = current_pose.pose.position
        #orientation = current_pose.pose.orientation

        #Access other feedback fields
        #navigation_time = feedback_msg.feedback.navigation_time
        #distance_remaining = feedback_msg.feedback.distance_remaining

#---------------------------------------------------------------------------------------------------------------------------------------------------------------------


    def callback(self, data):

        # Convert the received image into a opencv image 
        # But remember that you should always wrap a call to this conversion method in an exception handler 
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('camera_Feed',320,240)

        
        height, width, _ = image.shape
        center_x = width // 2
        

        # Set the upper and lower bounds for the two colours you wish to identify
        # The red hue value is around 0 or 180 and is at the boundary of the HSV requiring the definition of two intervals
        hsv_red_lower1 = np.array([0 - self.sensitivity, 100, 100])
        hsv_red_upper1 = np.array([0 + self.sensitivity, 255, 255])
        hsv_red_lower2 = np.array([180 - self.sensitivity, 100, 100])
        hsv_red_upper2 = np.array([180, 255, 255])  


        # The green hue value is 60
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])


        # The blue hue value is 110
        hsv_blue_lower = np.array([110 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([110 + self.sensitivity, 255, 255])


        # Convert the rgb image into a hsv image 
        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


        # Filter out everything but a particular colour using the cv2.inRange() method  
        mask_red_1 = cv2.inRange(Hsv_image, hsv_red_lower1, hsv_red_upper1)
        mask_red_2 = cv2.inRange(Hsv_image, hsv_red_lower2, hsv_red_upper2)
        mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)
        
        mask_green= cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
        
        mask_blue = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)


        # Apply the mask to the original image using the cv2.bitwise_and() method 
        # As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter 
        red_result = cv2.bitwise_and(image, image, mask=mask_red)

        green_result = cv2.bitwise_and(image, image, mask=mask_green)

        blue_result = cv2.bitwise_and(image, image, mask=mask_blue)


        # Find the contours that appear within the certain colour mask using the cv2.findContours() method 
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE 
        red_contours, red_hierarchy = cv2.findContours(mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        green_contours, green_hierarchy = cv2.findContours(mask_green, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        blue_contours, blue_hierarchy = cv2.findContours(mask_blue, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)


        # 定义最小面积阈值
        area_threshold = 300

        # red
        if len(red_contours) > 0:
                c = max(red_contours, key=cv2.contourArea)
                if cv2.contourArea(c) > area_threshold:
                        self.red_detected = True
                        cv2.drawContours(image, [c], -1, (0, 0, 255), 2)
                        M = cv2.moments(c)
                        if M["m00"] != 0:
                                cX = int(M["m10"] / M["m00"])
                                cY = int(M["m01"] / M["m00"])
                                cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)  # #the centre point of red block

        #green
        if len(green_contours) > 0:
                c = max(green_contours, key=cv2.contourArea)
                if cv2.contourArea(c) > area_threshold:
                        self.green_detected = True
                        cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                        M = cv2.moments(c)
                        if M["m00"] != 0:
                                cX = int(M["m10"] / M["m00"])
                                cY = int(M["m01"] / M["m00"])
                                cv2.circle(image, (cX, cY), 5, (0, 255, 0), -1)  #the centre point of green block



        #blue
        if len(blue_contours) > 0:
                c = max(blue_contours, key=cv2.contourArea)
                if cv2.contourArea(c) > area_threshold:
                        self.blue_detected = True
                        cv2.drawContours(image, [c], -1, (255, 0, 0), 2)


                        x, y, w, h = cv2.boundingRect(c)
                        rect_center_x = x +(w // 2)
                        rect_center_y = y +(h // 2)

                        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 255), 3) #'yellow lock box' if see blue block
                        cv2.circle(image, (rect_center_x, rect_center_y), 8, (0, 255, 255), -1) #the centre point of blue block

                        offset = rect_center_x - center_x
                        angle_tolerance = 30



                        #print(rect_center_x)
                        #print(center_x)
                        if offset > angle_tolerance:
                                #if the centre of blue block is on the right side of camera, then turn right
                                self.turn_left_flag = False
                                self.turn_right_flag = True
                        elif offset < -angle_tolerance:
                                #if the centre of blue block is on the left side of camera, then turn left
                                self.turn_left_flag = True
                                self.turn_right_flag = False
                        else:
                                self.turn_left_flag = False
                                self.turn_right_flag = False


                        aValue = 230000
                        tolerance = 30000

                        if cv2.contourArea(c) > aValue + tolerance:
                                self.move_forwards_flag = False
                                self.move_backwards_flag = True
                                self.stop_flag = False
                                
                        elif cv2.contourArea(c) < aValue - tolerance:
                                self.move_forwards_flag = True
                                self.move_backwards_flag = False
                                self.stop_flag = False
                                
                        else:
                                self.move_forwards_flag = False
                                self.move_backwards_flag = False
                                self.stop_flag = True

        # Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        cv2.imshow('camera_Feed', image)
        cv2.waitKey(3)


    def walk_forward(self):
        #Use what you learnt in lab 3 to make the robot move forwards 
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2


        for _ in range(30):  # Stop for a brief moment 
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def walk_backward(self):
        # Use what you learnt in lab 3 to make the robot move backwards 
        desired_velocity = Twist()
        desired_velocity.linear.x = - 0.2



        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()


    def stop(self):
        # Use what you learnt in lab 3 to make the robot stop
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0
        desired_velocity.angular.z = 0.0


        self.publisher.publish(desired_velocity)


    def turn_left(self):
        twist = Twist()
        twist.angular.z = 0.2  # turn left
        #for _ in range(30):  # Stop for a brief moment
        self.publisher.publish(twist)
        #self.rate.sleep()
        

    def turn_right(self):
        twist = Twist()
        twist.angular.z = -0.2  # turn right
        #for _ in range(30):  # Stop for a brief moment
        self.publisher.publish(twist)
        #self.rate.sleep()




# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()


    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    robot = Robot()
    


    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    
    try:
        #Navigation
        goals = [
                (-0.103, -2.45, 0.0),
                (-1.24, -4.96, 0.0),
                (-2.45, -4.3, 0.00),
                (1.48, -9.38, 0.0),
                (-3.43, -9.14, 0.0),  #coordinate of the centre of blue block(final point)
                ]
        for idx, (x, y, yaw) in enumerate(goals):
            robot.navigation_done_event.clear()
            robot.get_logger().info(f'moving to goal point {idx + 1}/{len(goals)}...')
            robot.send_next_goal(x, y, yaw)
            robot.navigation_done_event.wait()  # wait

        robot.get_logger().info("all goal points finifhed")

        #Vision 
        while rclpy.ok():
                #print("HERE")
                if robot.turn_left_flag:
                        robot.turn_left()
                elif robot.turn_right_flag:
                        robot.turn_right()
                elif robot.move_forwards_flag:
                        robot.walk_forward()
                elif robot.move_backwards_flag:
                        robot.walk_backward()
                else:
                        robot.stop()
                time.sleep(0.1)

    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
