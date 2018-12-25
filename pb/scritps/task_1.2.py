#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import    Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32

class WayPoint:

    def __init__(self):
        rospy.init_node('ros_bridge')
        self.image_pub = rospy.Publisher("topic",Image)
        self.cam = cv2.VideoCapture(1)

        # Create a ROS Bridge
        self.ros_bridge = cv_bridge.CvBridge()



    def image_callback(self):

        # 'image' is now an opencv frame
        # You can run opencv operations on 'image'



        cv2.namedWindow("test")


        while True:
            ret, frame = self.cam.read()
            cv2.imshow("test", frame)

            key = cv2.waitKey(10)
            if key == 27:
                break




            cv2.imshow("img_name", frame)
            self.image_pub.publish(self.ros_bridge.cv2_to_imgmsg(frame, "bgr8"))

        cv2.waitKey(0)
        cam.release()

        cv2.destroyAllWindows()




if __name__ == '__main__':
    test = WayPoint()

    test.image_callback()
    rospy.spin()
