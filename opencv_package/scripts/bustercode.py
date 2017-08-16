#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('opencv_package')
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import rospy
from geometry_msgs.msg import Twist  # Message that move the


class ControlBebop():
    def __init__(self):
        global current_angle_y
	# ControlBebop is the name of the node sent to the master
        #rospy.init_node('ControlBebop', anonymous=False)

        # Message to screen
        rospy.loginfo(" Press CTRL+c to stop Bebop")
        # Keys CNTL + c will stop script
        rospy.on_shutdown(self.shutdown)

        # Publisher will send Twist message on topic /bebop/camera_control

        self.cmd_vel = rospy.Publisher('/bebop/camera_control', Twist, queue_size=10)
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback)

        # TurtleBot will receive the message 10 times per second.
        rate = rospy.Rate(10);
        # 10 Hz is fine as long as the processing does not exceed 1/10 second.

        # Twist is a type of geometry_msgs for linear and angular velocity
        # Pan the camera by sending command to angular.y
        pan_camera_cmd = Twist()
        # Linear speed in x in meters/second is + (forward) or - (backwards)
        '''current_angle_y = camera_angle_correction(tag_center(corners), current_angle_y)
        print(current_angle_y)
        pan_camera_cmd.angular.y =  current_angle_y # Modify this value to change speed
        '''
        # tilt the camera
        #tilt_camera_cmd = Twist()
        #tilt_camera_cmd.angular.z = 20  # Modify this value to cause rotation rad/s

        # Loop and TurtleBot will move until you type CNTL+c
        #while not rospy.is_shutdown():
            # publish the Twist values to the TurtleBot node /cmd_vel_mux
        '''     rospy.loginfo("Pan the camera")
        for i in range(10):
           self.cmd_vel.publish(pan_camera_cmd)
                # wait for 0.1 seconds (10 HZ) and publish again
           rate.sleep()'''

    def tag_center(self, corners):

        aruco_code_center_x = (corners[0][0][0][0] + corners[0][0][2][0]) / 2
        aruco_code_center_y = (corners[0][0][0][1] + corners[0][0][2][1]) / 2
        aruco_code_center_pixel = [aruco_code_center_x, aruco_code_center_y]
        return aruco_code_center_pixel

    # camera_angle_correction takes the ArUco code center pixel position and the current angle of the camera
    # calculates whether we need to pan up or down to move the ArUco tag to the center of the screen
    # returns the new angle we need to publish to the topic
    def camera_angle_correction(self, aruco_code_center_pixel, current_angle_y):

        position_error_x = 488 - aruco_code_center_pixel[0]
        position_error_y = 240 - aruco_code_center_pixel[1]

        # Does some bounds checking as +/- 70 is maximum angle
        if ((position_error_y > 0) and (current_angle_y <= 65) and (current_angle_y >= -65)):
            new_angle_y = current_angle_y + 5

        elif ((position_error_y < 0) and (current_angle_y <= 65) and (current_angle_y >= -65)):
            new_angle_y = current_angle_y - 5

        else:
            new_angle_y = current_angle_y

        return new_angle_y

    def shutdown(self):
        # You can stop turtlebot by publishing an empty Twist message
        rospy.loginfo("Stopping Bebop")
        #
        self.cmd_vel.publish(Twist())
        # Give TurtleBot time to stop
        rospy.sleep(1)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()

        # lists of ids and the corners beloning to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        gray = aruco.drawDetectedMarkers(gray, corners)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    current_angle_y = 0
    print("start")
    cb = ControlBebop
    rospy.init_node('ControlBebop', anonymous=False)
    try:
        rospy.spin()
        print("spin")
    except KeyboardInterrupt:
        rospy.loginfo("End of the trip for Bebop")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)