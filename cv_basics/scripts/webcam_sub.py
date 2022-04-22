#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import math
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import apriltag

def callback(data):
    # Used to convert between ROS and OpenCV images
    bridge = CvBridge()

    # Output debugging information to the terminal
    rospy.loginfo("receiving video frame")

    # Convert ROS Image message to OpenCV image
    current_frame = bridge.imgmsg_to_cv2(data)

    gray = cv2.cvtColor(current_frame,cv2.COLOR_BGR2GRAY)

    # define the AprilTags detector options and then detect the AprilTags
    # in the input image
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    image = detector.detect(gray)

    # d = (w * f) / p
    # w - actual width
    # f - focal length
    # p - pixel width on camera image

    # Webcam Approx. Focal Length = 76 mm (distance) * 342 px (pixel width)
    # / 51 mm (real length
    # of april tag)
    # 351 px x 342 px x 342 px x 346 px
    # RealSense Focal Length - 1.93 mm

    real_width = 0.051 # m

    focal_length = 0.076 * (342 / 0.051)

    for r in image:
        (point_a, point_b, point_c, point_d) = r.corners
        point_b = (int(point_b[0]), int(point_b[1]))
        point_c = (int(point_c[0]), int(point_c[1]))
        point_d = (int(point_d[0]), int(point_d[1]))
        point_a = (int(point_a[0]), int(point_a[1]))

        side_len_1 = math.dist(point_a, point_b)
        side_len_2 = math.dist(point_b, point_c)
        side_len_3 = math.dist(point_c, point_d)
        side_len_4 = math.dist(point_d, point_a)

        cv2.line(current_frame, point_a, point_b, (0, 255, 0), 2)
        cv2.line(current_frame, point_b, point_c, (0, 255, 0), 2)
        cv2.line(current_frame, point_c, point_d, (0, 255, 0), 2)
        cv2.line(current_frame, point_d, point_a, (0, 255, 0), 2)

        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(current_frame, (cX, cY), 5, (0, 0, 255), -1)

        side_len_array = (side_len_1, side_len_2, side_len_3, side_len_4)
        pixel_width = max(side_len_array)
        distance = str((real_width * focal_length)/(pixel_width))
        cv2.putText(current_frame, distance, (point_a[0], point_a[1] - 15),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2, cv2.LINE_AA)

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
