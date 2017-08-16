#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy
import tf
from geometry_msgs.msg import PoseStamped,Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Empty 
from pid import PID


class Detector():

   """ Detector for the target -- identified by pink marker """
   """ Target's location is published as a PoseStamped message with its x, y, z with respect to  """
   """   the Kinect v2's image """

   def __init__(self):

      # initialize ROS node
      rospy.init_node('target_detector', anonymous=True)

      # initialize publisher for target pose, PoseStamped message, and set initial sequence number
      self.pub = rospy.Publisher('target_pose', PoseStamped, queue_size=1)
      self.pub_pose = PoseStamped()
      self.pub_pose.header.seq = 0

      # Setup Publishers
      self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10) 
      self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10) 
      self.fly_control_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
      self.camera_control_pub = rospy.Publisher('/bebop/camera_control', Twist, queue_size=10)

      # Setup command type
      self.takeoff_cmd = Empty()                      # Takeoff
      self.land_cmd = Empty()                         # Land
      self.camera_control_cmd = Twist()               # Camera Control
      self.fly_control_cmd = Twist()                  # fly Velocity Control

      self.rate = rospy.Rate(1.0)                      # publish message at 1 Hz

      # Initialize values for locating target on Bebop camera image 
      self.target_u = 0                        # u is pixels left(0) to right(+) 0 to 856
      self.target_v = 0                        # v is pixels top(0) to bottom(+) 0 to 480
      self.target_d = 0                        # d is distance camera(0) to target(+) from depth image
      self.target_found = False                # flag initialized to False
      self.last_d = 0                          # last non-zero depth measurement

      # Initialize camera angle
      self.camera_angle_x = 0                  # vertial pan -70 to 17 degrees
      self.camera_angle_y = 0                  # horizontal pan -35 to 35 degrees
      self.pidX = PID(0.005, 0.00, 0.0, -3, 3, -0.1, 0.1)
      self.pidY = PID(0.005, 0.0, 0.00, -2, 4, -0.1, 0.1)

      # Initialize horizontal and vertical velocities 
      self.pitch_vel = 0                 # linear.x backward(-1) to forward(+1) 
      self.roll_vel = 0                  # linear.y right(-1) to left(+1)
      self.vertical_vel = 0              # linear.z descend(-1) to ascend(+1)
      self.rotate_vel = 0                # angular.z clockwise(-1) to anti-clockwise(+1)
      self.pid_Pitch = PID(0.005, 0.00, 0.0, -0.02, 0.02, -0.1, 0.1)
      self.pid_Roll = PID(0.005, 0.00, 0.0, -0.02, 0.02, -0.1, 0.1)
      self.pid_Vertical = PID(0.005, 0.00, 0.0, -0.02, 0.02, -0.1, 0.1)
      self.pid_Rotate = PID(0.001, 1.00, 0.001, -0.02, 0.02, -0.1, 0.1)
      # target orientation to Kinect v2 image (Euler)
      self.r = 0
      self.p = 0
      self.y = 0

      # Convert image from a ROS image message to a CV image
      self.bridge = CvBridge()

      # Wait for the camera_info topic to become available
      rospy.wait_for_message('/bebop/image_raw', Image)

      # Subscribe to registered color and depth images
      rospy.Subscriber('/bebop/image_raw', Image, self.image_callback, queue_size=1)
      # rospy.Subscriber('/bebop/image_raw', Image, self.depth_callback, queue_size=1)

      self.rate.sleep()                        # suspend until next cycle

   # This callback function handles processing Kinect color image, looking for the pink target.
   def image_callback(self, msg):

      # convert ROS image to OpenCV image
      try:
         image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
      except CvBridgeError as e:
         print(e)

      # create hsv image of scene
      hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

      # find pink objects in the image
      lower_pink = numpy.array([0, 200, 121], numpy.uint8)
      upper_pink = numpy.array([19, 255, 221], numpy.uint8)
      mask = cv2.inRange(hsv, lower_pink, upper_pink)

      # dilate and erode with kernel size 11x11
      cv2.morphologyEx(mask, cv2.MORPH_CLOSE, numpy.ones((11, 11)))

      # find all of the contours in the mask image
      ''' Changes'''
      # ' _,'has been added before 'contours, heirarchy ....' because cv2.findContours() function
      # retrun 3 variables
      _, contours, heirarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      self.contourLength = len(contours)
      # print('countour length is ',self.contourLength)
      # Check for at least one target found
      if self.contourLength < 5:
         print "No target found"
	 self.fly_control_cmd.linear.x = 0
	 self.fly_control_cmd.linear.y = 0
	 self.fly_control_cmd.linear.z = 0
	 self.fly_control_cmd.angular.z = 0
         self.fly_control_pub.publish(self.fly_control_cmd)

      else:                       # target found

         # Loop through all of the contours, and get their areas
         area = [0.0] * len(contours)
         for i in range(self.contourLength):
            area[i] = cv2.contourArea(contours[i])
	    #rospy.loginfo('area is %d',  max(area))
         # Target #### the largest "pink" object
         target_image = contours[area.index(max(area))]

         # Using moments find the center of the object and draw a red outline around the object
         target_m = cv2.moments(target_image)
         self.target_u = int(target_m['m10'] / target_m['m00'])
         self.target_v = int(target_m['m01'] / target_m['m00'])
         points = cv2.minAreaRect(target_image)
         '''Changes'''
         # change 'box = cv2.cv.BoxPoints(points)' to 'box = cv2.boxPoints(points)' as follows
         # the syntax was wrong
         box = cv2.boxPoints(points)
         box = numpy.int0(box)
         cv2.drawContours(image, [box], 0, (0, 0, 255), 2)
	 cv2.circle(image, (428, 240), 10, (255, 0, 0), -1)
         cv2.circle(image, (self.target_u, self.target_v), 5, (255, 255, 255), 2)
         rospy.loginfo("Center of target is x at %d and y at %d", int(self.target_u), int(self.target_v))
	 # update the angle 
	 #self.camera_angle_x -= self.pidX.update(self.target_u, 428)
         #self.camera_angle_y += self.pidY.update(self.target_v, 240)
	 # Publish the command through the Publisher
         #self.camera_control_cmd.angular.z = self.camera_angle_x
         #self.camera_control_cmd.angular.y = self.camera_angle_y
         #self.camera_control_pub.publish(self.camera_control_cmd)
         #rospy.loginfo("camera_angle_x: %d camera_angle_y: %d", self.camera_angle_x, self.camera_angle_y)

	 self.rotate_vel += self.pid_Rotate.update(self.target_u, 428)
         #self.roll_vel += self.pid_Roll.update(self.target_u, 428)
	 #self.vertical_vel += self.pid_Vertical.update(self.target_v, 240)
	 
	 #self.fly_control_cmd.linear.y = min(max(-0.2, self.roll_vel), 0.2)
	 #self.fly_control_cmd.linear.z = min(max(-0.2, self.vertical_vel), 0.2)
	 self.fly_control_cmd.angular.z = min(max(-0.2, self.rotate_vel), 0.2)
	 self.fly_control_pub.publish(self.fly_control_cmd)
	 
	 #self.camera_control()
         self.target_found = True               # set flag for depth_callback processing

      # show image with target outlined with a red rectangle
      # cv2.imshow("Mask", mask)
      cv2.imshow("Target", image)
      cv2.waitKey(2)
   def camera_control(self):
      # update the angle 
      self.camera_angle_x -= self.pidX.update(self.target_u, 428)
      self.camera_angle_y += self.pidY.update(self.target_v, 240)
      # Publish the command through the Publisher
      self.camera_control_cmd.angular.z = self.camera_angle_x
      self.camera_control_cmd.angular.y = self.camera_angle_y
      self.camera_control_pub.publish(self.camera_control_cmd)
      rospy.loginfo("camera_angle_x: %d camera_angle_y: %d", self.camera_angle_x, self.camera_angle_y)
	
   # This callback function handles processing Kinect depth image, looking for the depth value
   #   at the location of the center of the pink target.
   def depth_callback(self, msg):

      # process only if target is found
      if self.target_found == True:

         # create OpenCV depth image using default passthrough encoding
         try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
         except CvBridgeError as e:
            print(e)

         # using target (v, u) location, find depth value of point and divide by 1000
         # to change millimeters into meters (for Kinect sensors only)
         self.target_d = depth_image[self.target_v, self.target_u] / 1000.0

         # if depth value is zero, use the last non-zero depth value
         if self.target_d == 0:
            self.target_d = self.last_d
         else:
            self.last_d = self.target_d

         # record target location and publish target pose message
         rospy.loginfo("Target depth: x at %d  y at %d  z at %f", int(self.target_u),
                       int(self.target_v), self.target_d)
         self.update_target_pose(self.target_u, self.target_v, self.target_d)

   # This function builds the target PoseStamped message and publishes it.
   def update_target_pose(self, x, y, z):

      # set position values for target location
      self.pub_pose.pose.position.x = x      # pose in camera pixels u
      self.pub_pose.pose.position.y = y      # pose in camera pixels v
      self.pub_pose.pose.position.z = z      # pose in meters from camera

      # determine quaternion values from euler angles of (0,0,0)
      quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
      self.pub_pose.pose.orientation.x = quaternion[0]
      self.pub_pose.pose.orientation.y = quaternion[1]
      self.pub_pose.pose.orientation.z = quaternion[2]
      self.pub_pose.pose.orientation.w = quaternion[3]

      # complete header information
      self.pub_pose.header.seq += 1
      self.pub_pose.header.stamp = rospy.Time.now()
      self.pub_pose.header.frame_id = "target"

      # publish pose of target
      self.pub.publish(self.pub_pose)

   def Debug(self):
      rospy.loginfo("Debug function is running")

if __name__ == '__main__':

   # start up the detector node and run until shutdown by interrupt
   try:
      detector = Detector()
      detector.Debug()
      rospy.spin()

   except rospy.ROSInterruptException:
      rospy.loginfo("Detector node terminated.")

   # close all terminal windows when process is shut down
   cv2.destroyAllWindows()
