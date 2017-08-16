#!/usr/bin/env python
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


# Currently need to find a place to store the camera_angle variable so that we can keep track of the current angle and the 
# camera_angle_correction function can be passed it.

# Also, need to understand the exact way to format the message we need to publish on the topic because it is currently throwing errors



class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)
    # Added the line below so we can publish messages to the camera_control topic with message type Twist
    self.message_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/bebop/image_raw",Image,self.callback)
    self.cmd_vel = rospy.Publisher('/bebop/camera_control', Twist, queue_size=10)
    self.pan_camera_cmd = Twist()

  # tag_center takes the values for the pixels of the corners of an identified ArUco tag and calculates the 
  # position of the center pixel based on these, returning the two element list with x and y positions
  def tag_center(self, corners):
  
    aruco_code_center_x = (corners[0][0][0][0] + corners[0][0][2][0])/2
    aruco_code_center_y = (corners[0][0][0][1] + corners[0][0][2][1])/2
    aruco_code_center_pixel = [aruco_code_center_x, aruco_code_center_y]
    return aruco_code_center_pixel

  # camera_angle_correction takes the ArUco code center pixel position and the current angle of the camera
  # calculates whether we need to pan up or down to move the ArUco tag to the center of the screen
  # returns the new angle we need to publish to the topic
  def camera_angle_correction(self,aruco_code_center_pixel, current_angle_y):

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


  # publish_the_message takes the camera_angle we want to publish and publishes it to the camera_control topic
  def publish_the_message(self, camera_angle):
    # pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
    #rospy.init_node('node2', anonymous=True)
    rate = rospy.Rate(10)


    while not rospy.is_shutdown():
      # string_to_publish = '{angular: {y : %d }}' % camera_angle
      self.pan_camera_cmd.angular.y = camera_angle
      self.cmd_vel.publish(self.pan_camera_cmd)
      
      print(camera_angle)
      rate.sleep()
 


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    global camera_angle

   #(rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)

    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

     #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
  
    '''if (len(corners) != 0):
  
      aruco_code_center_pixel = self.tag_center(corners)    
      camera_angle = self.camera_angle_correction(aruco_code_center_pixel, camera_angle)
      self.publish_the_message(camera_angle)'''

    gray = aruco.drawDetectedMarkers(gray, corners)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      if (len(corners) != 0):
          aruco_code_center_pixel = self.tag_center(corners)
          camera_angle = self.camera_angle_correction(aruco_code_center_pixel, camera_angle)
          self.publish_the_message(camera_angle)
    except CvBridgeError as e:
      print(e)


camera_angle = 0
def main(args):
 

  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
