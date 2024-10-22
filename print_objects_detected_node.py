#!/usr/bin/env python
'''
功能：该脚本订阅图像和对象信息，并在图像上绘制检测到的对象。
转换细节：
使用 cv2.perspectiveTransform 计算变换后的点。
使用 cv_bridge 将 ROS 图像消息转换为 OpenCV 格式。
使用 message_filters 实现同步订阅。
使用说明：
确保已安装 ROS 和相关依赖库（如 OpenCV 和 cv_bridge）。
将此文件保存为 objects_detected.py 并赋予可执行权限：chmod +x objects_detected.py。
在 ROS 环境中运行此脚本。
'''
import rospy
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from find_object_2d.msg import ObjectsStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

bridge = CvBridge()
image_pub = None

def objects_detected_callback(msg):
    rospy.loginfo("---")
    data = msg.data
    if data:
        for i in range(0, len(data), 12):
            object_id = int(data[i])
            object_width = data[i + 1]
            object_height = data[i + 2]

            # 计算角点
            homography = np.array(data[i + 3:i + 12]).reshape(3, 3)
            points = np.array([[0, 0], [object_width, 0], [object_width, object_height], [0, object_height], [object_width / 2, object_height / 2]])
            transformed_points = cv2.perspectiveTransform(points.reshape(-1, 1, 2), homography)

            rospy.loginfo("Object %d detected, corners at %s", object_id, transformed_points.squeeze())
    else:
        rospy.loginfo("No objects detected.")

def image_objects_detected_callback(image_msg, objects_msg):
    if image_pub.get_num_subscribers() > 0:
        data = objects_msg.objects.data
        if data:
            for i in range(0, len(data), 12):
                object_id = int(data[i])
                object_width = data[i + 1]
                object_height = data[i + 2]

                # 计算角点
                homography = np.array(data[i + 3:i + 12]).reshape(3, 3)
                points = np.array([[0, 0], [object_width, 0], [object_width, object_height], [0, object_height], [object_width / 2, object_height / 2]])
                transformed_points = cv2.perspectiveTransform(points.reshape(-1, 1, 2), homography)

                # 处理图像
                cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
                transformed_points_int = transformed_points.astype(int)

                # 绘制多边形
                cv2.polylines(cv_image, [transformed_points_int], isClosed=True, color=(0, 255, 0), thickness=3)
                center = transformed_points[4].astype(int)
                cv2.putText(cv_image, f"({center[0]}, {center[1]})", tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                cv2.circle(cv_image, tuple(center), 5, (255, 0, 0), -1)

                image_pub.publish(bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

def main():
    global image_pub
    rospy.init_node('objects_detected')
    image_pub = rospy.Publisher("image_with_objects", Image, queue_size=1)

    # 订阅对象和图像
    image_sub = message_filters.Subscriber("image", Image)
    objects_sub = message_filters.Subscriber("objectsStamped", ObjectsStamped)
    ts = message_filters.TimeSynchronizer([image_sub, objects_sub], 10)
    ts.registerCallback(image_objects_detected_callback)

    rospy.Subscriber("objects", Float32MultiArray, objects_detected_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
