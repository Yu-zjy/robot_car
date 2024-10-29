#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePreprocessor:
    def __init__(self):
        rospy.init_node('image_preprocessor')
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("processed_image", Image, queue_size=1)
        
        self.image_sub = message_filters.Subscriber("/usb_cam/image_raw", Image)
        self.image_sub.registerCallback(self.image_callback)

    def image_callback(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  
        blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)  
        _, binary_image = cv2.threshold(blurred_image, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU) 

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(binary_image, encoding='mono8'))

if __name__ == '__main__':
    try:
        ImagePreprocessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
