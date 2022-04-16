#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import apriltag
  
def callback(data):
  
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
  
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
  
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data)
  
  gray = cv2.cvtColor(current_frame,cv2.COLOR_BGR2GRAY)
      
  # define the AprilTags detector options and then detect the AprilTags
  # in the input image
  options = apriltag.DetectorOptions(families="tag36h11")
  detector = apriltag.Detector(options)
  image = detector.detect(gray)
  
  # Webcam Approx. Focal Length = 76 mm (distance) * 342 px (pixel width) / 51 mm (real length
  # of april tag)
  # 351 px x 342 px x 342 px x 346 px
  # RealSense Focal Length - 1.93 mm
  
  real_width = 0.051 # m
  
  focal_length = 0.076 * (342 / 0.051)
    
  for r in image:
  	(ptA, ptB, ptC, ptD) = r.corners
  	ptB = (int(ptB[0]), int(ptB[1]))
  	ptC = (int(ptC[0]), int(ptC[1]))
  	ptD = (int(ptD[0]), int(ptD[1]))
  	ptA = (int(ptA[0]), int(ptA[1]))
  	
  	cv2.line(current_frame, ptA, ptB, (0, 255, 0), 2)
  	cv2.line(current_frame, ptB, ptC, (0, 255, 0), 2)
  	cv2.line(current_frame, ptC, ptD, (0, 255, 0), 2)
  	cv2.line(current_frame, ptD, ptA, (0, 255, 0), 2)
      
  	(cX, cY) = (int(r.center[0]), int(r.center[1]))
  	cv2.circle(current_frame, (cX, cY), 5, (0, 0, 255), -1)
  	
  	# d = (w * f) / p
  	# w: actual width
  	# f: focal length
  	# p: pixel width on camera image
  	
  	# pixel_width = . . .
  	
  	distance = (real_width * focal_length)/(pixel_width)

  # Display image
  cv2.imshow("camera", current_frame)
  
  cv2.waitKey(1)
  
def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()
