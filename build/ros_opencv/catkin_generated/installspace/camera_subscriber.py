#!/usr/bin/env python3
# here, we are saying to ROS that this is a Python source file

# Here we import rospy that enables us to use ROS with Python
import rospy

# we will send messages in form of images consequently, we need to import Image 
from sensor_msgs.msg import Image 

# cv_bridge is a package that consists of a library for converting OpenCV images (of type cv::Mat)
# into a ROS image message and for converting ROS image message back to OpenCV images
# That is, it serves as a bridge between OpenCV and ROS
from cv_bridge import CvBridge 

# here we import OpenCV
import cv2 

# create the name of our publisher node - change as you wish
subscriberNodeName='camera_sensor_subscriber'


# make sure that the same name is used in the source file of the publisher
topicName='video_topic'



# this function is a callback function that is called every time the message arrives
def callbackFunction(message):
	# create a bridge object
	bridgeObject=CvBridge()
	# print the message
	rospy.loginfo("received a video message/frame")
	# convert from cv_bridge to OpenCV image format
	convertedFrameBackToCV=bridgeObject.imgmsg_to_cv2(message)
	
	# show the image on the screen
	cv2.imshow("camera",convertedFrameBackToCV)
	
	# wait in miliseconds, here only 1 milisecond
	cv2.waitKey(1)
	

# initialize the subscriber node, 
# anonymous=True means that a random number is added to the subsriber node name 	
rospy.init_node(subscriberNodeName, anonymous=True)
# here we subscribe: specify the topic name, type of the message we will receive,
# and the name of the callback function
rospy.Subscriber(topicName,Image, callbackFunction)
# here we "spin" the code, meaning that we execute it infinitely, until we press ctrl+c
rospy.spin()
# destroy all the windows
cv2.destroyAllWindows()





























