# robosys-apriltag

***Note:*** This tutorial is based heavily on <a href="https://automaticaddison.com/working-with-ros-and-opencv-in-ros-noetic/" target="_blank">this one by Automatic Addison</a>.

## Instructions

Having installed ROS and Python, this tutorial should work.

Download and drop the ***cv_basics*** folder in your ***catkin_ws/src*** folder.

Then in a shell, ***pip install image_transport cv_bridge sensor_msgs rospy roscpp std_msgs*** (will debug this in a bit).

Then redirect to the ***catkin_ws/src/cv_basics/scripts*** directory, and enter ***chmod +x webcam_pub.py*** and ***chmod +x webcam_sub.py*** to make them executables.

Finally, enter ***roslaunch cv_basics cv_basics_py.launch*** , and it should work!

___________________________________________________________________________________________________________________________________________________________

(Note: one of the dependencies, image_transport's name has been changed, will check and debug later)
