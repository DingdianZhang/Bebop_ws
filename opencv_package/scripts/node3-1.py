#!/usr/bin/env python

'''
This program causes the camera on the Bebop to pan in the Y direction so that the tag is in the middle
of the screen.
'''
from __future__ import print_function
import roslib
roslib.load_manifest('opencv_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
from geometry_msgs.msg import Twist
import glob
import numpy as np
from pid import PID


class image_converter:

  def __init__(self):
    # Image after use in cv2 can be published
    self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
    # Added the line below so we can publish messages to the camera_control topic with message type Twist
    self.message_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
    self.bridge = CvBridge()
    # Subscribe to the image_raw topic, and use the callback function
    self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback)
    self.cmd_vel = rospy.Publisher('/bebop/camera_control', Twist, queue_size=10)
    self.pan_camera_cmd = Twist()
    self.pidX = PID(0.1, 0.1, 0.1, -2.0, 4.0, -0.1, 0.1)
    self.pidY = PID(0.1, 0.1, 0.1, -2.0, 4.0, -0.1, 0.1)
  # tag_center takes the values for the pixels of the corners of an identified ArUco tag and calculates the
  # position of the center pixel based on these, returning the two element list with x and y positions

  def tag_center(self, corners):

    aruco_code_center_x = (corners[0][0][0][0] + corners[0][0][2][0]) / 2
    aruco_code_center_y = (corners[0][0][0][1] + corners[0][0][2][1]) / 2
    aruco_code_center_pixel = [aruco_code_center_x, aruco_code_center_y]
    rospy.loginfo("tag_center_x: %d tag_center_y: %d", aruco_code_center_x, aruco_code_center_y)
    return aruco_code_center_pixel

  # camera_angle_correction takes the ArUco code center pixel position and the current angle of the camera
  # calculates whether we need to pan up or down to move the ArUco tag to the center of the screen
  # returns the new angle we need to publish to the topic
  def camera_angle_correction(self, aruco_code_center_pixel, current_angle_x, current_angle_y):

    position_error_x = 428 - aruco_code_center_pixel[0]
    position_error_y = 240 - aruco_code_center_pixel[1]

    new_angle_x = current_angle_x
    # Does some bounds checking as +/- 70 is maximum angle
    if ((position_error_x > 0) and (current_angle_x <= 35) and (current_angle_x >= -35)):
      if current_angle_x == 35:
        new_angle_x = current_angle_x
      else:
        new_angle_x = current_angle_x - 1

    elif ((position_error_x < 0) and (current_angle_x - 1 <= 35) and (current_angle_x - 1 >= -35)):
      if current_angle_x == -35:
        new_angle_x = current_angle_x
      else:
        new_angle_x = current_angle_x + 1

    # If the new_angle_y is going to go out of bounds, set it equal to the current angle
    else:
      new_angle_y = current_angle_y

    if ((position_error_y > 0) and (current_angle_y <= 17) and (current_angle_y >= -70)):
      if current_angle_y == 17:
        new_angle_y = current_angle_y
      else:
        new_angle_y = current_angle_y + 1

    elif ((position_error_y < 0) and (current_angle_y - 1 <= 17) and (current_angle_y - 1 >= -70)):
      if current_angle_y == -70:
        new_angle_y = current_angle_y
      else:
        new_angle_y = current_angle_y - 1

    # If the new_angle_y is going to go out of bounds, set it equal to the current angle
    else:
      new_angle_y = current_angle_y

    return new_angle_x, new_angle_y

  # publish_the_message takes the camera_angle we want to publish and publishes it to the camera_control topic
  def publish_the_message(self, camera_angle_x, camera_angle_y):
      # Only changes the y plane values, we could additionally add in x also
    self.pan_camera_cmd.angular.z = camera_angle_x
    self.pan_camera_cmd.angular.y = camera_angle_y
    self.cmd_vel.publish(self.pan_camera_cmd)
    rospy.loginfo("camera_angle_x: %d camera_angle_y: %d", camera_angle_x, camera_angle_y)

  # draw the coordinate lines on the image
  def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img

  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    global camera_angle_x, camera_angle_y
    markerLength = 5

    #(rows,cols,channels) = cv_image.shape
    # if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)

    #cv2.imshow("Image window", cv_image)
    # cv2.waitKey(3)

    # Load the camera coefficients etc
    with np.load('B.npz') as X:
      mtx, dist, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]

    # Make the image gray for ArUco tag detection
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # Assign the dictionary we wish to use (6 bit, with 250 separate tags)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    # lists of ids and the corners belonging to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    #aruco.drawDetectedMarkers(cv_image, corners, ids)

    # this if statement lets us check if we have detected a tag by checking the size
    # of the corners list to see if it's greater than 0. If it is, then we want to
    # find the center position of the tag and then correct the camera angle to center
    # it by publishing back a new angle to the control_camera topic.
    if (len(corners) != 0):
      # Draw on the markers so we can see they've been detected
      gray = aruco.drawDetectedMarkers(cv_image, corners, ids)
      rvec, tvec = aruco.estimatePoseSingleMarkers(corners, markerLength, mtx, dist)  # For a single marker
      gray = aruco.drawAxis(gray, mtx, dist, rvec, tvec, 10)
      aruco_code_center_pixel = self.tag_center(corners)
      camera_angle_x, camera_angle_y = self.camera_angle_correction(aruco_code_center_pixel, camera_angle_x, camera_angle_y)

      self.publish_the_message(camera_angle_x, camera_angle_y)  # use this method to publish the desired camera angle the the /bebop/camera_control topic

    # Display the video feed frames every 3 ms.
    cv2.imshow("Image window", cv_image)
    cv2.imshow("Gray window", gray)
    cv2.waitKey(5)

    # Publish the image back into ROS image message type (not sure why, I guess it's if you
    # want to do something else with it after using OpenCV).
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    except CvBridgeError as e:
      print(e)

# Setup an initial camera angle
camera_angle_x = 0
camera_angle_y = 0


def main(args):
  # Initialise the node under the name image_converter
  rospy.init_node('image_converter', anonymous=True)
  # Assign the ic variable to the class type of image_converter
  ic = image_converter()
  # Keep running through until we stop it.
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
