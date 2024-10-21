#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pytesseract

class ImageProcessor:
    def __init__(self):
        rospy.init_node('image_processor')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 将图像转换为灰度图像
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 进行繁体中文识别
        text = pytesseract.image_to_string(gray_image, lang='chi_tra')  # 使用繁体中文识别
        rospy.loginfo(f"识别到的汉字: {text}")

if __name__ == '__main__':
    try:
        processor = ImageProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
