#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
# from std_msgs.msg import String, Float32
import cv2
import numpy as np
from boxes_3D.srv import boxes3D
from yolov8_ros.srv import Yolov8

class PointAtNode:

    def __init__(self):
        rospy.init_node("tester",anonymous=True)

        # self.image_sub = rospy.Subscriber('/kinect2/hd/image_color', Image, self.image_callback)
        # self.point_at_pub = rospy.Publisher('point_at_frame',Image,queue_size=10)
        # self.point_cloud_sub = rospy.Subscriber('/kinect2/hd/image_depth_rect', Image, self.image_depth_callback)
        # string model_name
        # uint8[] classes #if empty, all classes are tracked
        self.model_name=""
        self.model_class=[]
        self.image=Image()

    def test(self):
        rospy.wait_for_service('boxes_3d_service')

        try:
            rospy.loginfo("Attente du service boxes 3D")

            get_boxes_espace = rospy.ServiceProxy('boxes_3d_service',boxes3D)  
            bbox_espace = get_boxes_espace(self.model_name,self.model_class,self.image,self.image).boxes3D
            rospy.loginfo(bbox_espace)

        except rospy.ServiceException as e:

                print("Erreur dans l'appel du service: %s"%e)
if __name__=="__main__":
    if __name__ == '__main__':
        a=PointAtNode()
        a.test()
        