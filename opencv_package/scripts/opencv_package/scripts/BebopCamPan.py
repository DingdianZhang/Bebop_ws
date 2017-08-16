#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist   #Message that move the

class ControlBebop():
	def __init__(self):
	    # ControlBebop is the name of the node sent to the master
	    rospy.init_node('BebopCamPan', anonymous=False)

	    # Message to screen
	    rospy.loginfo(" Press CTRL+c to stop Bebop")
	    # Keys CNTL + c will stop script   
	    rospy.on_shutdown(self.shutdown)
	
	    #Publisher will send Twist message on topic /bebop/camera_control
	       
	    self.cmd_vel = rospy.Publisher('/bebop/camera_control', Twist, queue_size=10)
	     
	    # TurtleBot will receive the message 10 times per second. 
	    rate = rospy.Rate(30);   
	    # 10 Hz is fine as long as the processing does not exceed 1/10 second.

	    # Twist is a type of geometry_msgs for linear and angular velocity
	    # Pan the camera by sending command to angular.y
	    pan_camera_cmd = Twist()
	    # Linear speed in x in meters/second is + (forward) or - (backwards)
	    pan_camera_cmd.angular.y = -70	# Modify this value to change speed

	    # tilt the camera
	    tilt_camera_cmd = Twist()
	    tilt_camera_cmd.angular.z = -30	# Modify this value to cause rotation rad/s

	    # Loop and TurtleBot will move until you type CNTL+c
	    while not rospy.is_shutdown():
		# publish the Twist values to the TurtleBot node /cmd_vel_mux
		#rospy.loginfo("Pan the camera")
		#for i in range(10):  
		#    self.cmd_vel.publish(pan_camera_cmd)
		    # wait for 0.1 seconds (10 HZ) and publish again
		#    rate.sleep()
		rospy.loginfo("Tilt the camera")
		# tilt_camera_cmd.angular.z = 0
		for j in range(60):
		    tilt_camera_cmd.angular.z += 1
		    print tilt_camera_cmd.angular.z
		    self.cmd_vel.publish(tilt_camera_cmd)
		    rate.sleep()
		for j in range(60):
		    tilt_camera_cmd.angular.z -= 1
		    print tilt_camera_cmd.angular.z
		    self.cmd_vel.publish(tilt_camera_cmd)
		    rate.sleep()
		

	def shutdown(self):
            # You can stop turtlebot by publishing an empty Twist message 
	    rospy.loginfo("Stopping Bebop")
	    # 
	    self.cmd_vel.publish(Twist())
	    # Give TurtleBot time to stop
	    #rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        ControlBebop()
    except:
        rospy.loginfo("End of the trip for Bebop")
