#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def image_callback(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)
        return

    # ここで画像処理を行う
    # 例：画像をグレースケールに変換
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # 画像を表示
    cv2.imshow("Image", gray_image)
    cv2.waitKey(1)

def main():
    rospy.init_node("image_recognition_node")
    image_topic = "/usb_cam/image_raw"  # カメラトピックの名前を設定
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
