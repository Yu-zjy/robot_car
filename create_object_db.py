#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import yaml
import os

class CreateObjectDB:
    def __init__(self):
        rospy.init_node('create_object_db')

        # 获取参数
        self.image_topic = rospy.get_param('~image')
        self.object_name = rospy.get_param('~name')
        self.objects_path = rospy.get_param('~objects_path')

        # 创建图像桥接
        self.bridge = CvBridge()

        # 订阅图像话题
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)

        # 确保对象数据库路径存在
        self.ensure_directory(self.objects_path)

    def ensure_directory(self, path):
        directory = os.path.dirname(path)
        if not os.path.exists(directory):
            os.makedirs(directory)

    def image_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 图像
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 在这里执行特征提取和数据库更新
        features = self.extract_features(cv_image)

        # 保存到 YAML 文件
        self.save_to_yaml(features)

    def extract_features(self, cv_image):
        # 在这里实现特征提取逻辑
        # 例如：使用 ORB 算法提取特征
        orb = cv2.ORB_create()
        keypoints, descriptors = orb.detectAndCompute(cv_image, None)

        # 返回特征描述符
        return {
            'name': self.object_name,
            'keypoints': keypoints,
            'descriptors': descriptors.tolist()  # 转换为可序列化的格式
        }

    def save_to_yaml(self, features):
        # 读取现有数据库
        if os.path.exists(self.objects_path):
            with open(self.objects_path, 'r') as f:
                object_db = yaml.safe_load(f) or {}
        else:
            object_db = {}

        # 更新数据库
        object_db[self.object_name] = features

        # 保存更新后的数据库
        with open(self.objects_path, 'w') as f:
            yaml.dump(object_db, f)

if __name__ == '__main__':
    try:
        CreateObjectDB()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
