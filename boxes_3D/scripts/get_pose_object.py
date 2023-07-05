#!/usr/bin/env python3

"""
This code gets a ROS message with bounding boxes coordinates for each object detected
by the mask RCNN and get the XYZ coordinates of the bounding box center
in order to generate a TF on it
Written by Thomas CURE and Simon ERNST
"""


import rospy
# from sensor_msgs.msg import PointCloud2
import os
# from sensor_msgs import convertPointCloud2ToPointCloud
# from sensor_msgs.point_cloud2 import read_points
import numpy
from cv_bridge import CvBridge# import ros_numpy as rosnp
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
# from darknet_ros_msgs.msg import BoundingBoxes

from yolov8_ros.srv import Yolov8
from boxes_3D_pose.srv import boxes3D_pose, boxes3DResponse_pose
# from Boxes.msg import boxes
from boxes_3D_pose.msg import Box3D_pose
from boxes_3D_pose.msg import Boxes3D_pose
# from tf_broadcaster.msg import DetectionCoordinates, PointCoordinates
from nav_msgs.msg import Odometry


class GetCenterCoordinates(object):
    def __init__(self):
        """
        Create an instance of GetCenterCoordinates Class.
        Setup the ROS node and the parameters used in the code
        """
        rospy.init_node("centercalcul",anonymous=True)
        self.bridge = CvBridge()
        self.depth_msg=Image()
        self.rate=rospy.Rate(30)
        self.center_coordinates=None
        self.topic_image= rospy.get_param("/topic_image","/kinect2/hd/image_color")
        self.topic_depth= rospy.get_param("/topic_depth","/kinect2/hd/image_depth_rect")

        self.boxes_src = rospy.Service('boxes_3d_pose_service', boxes3D_pose, self.handle_bbox_req)
        rospy.loginfo("subscribed to"+" "+str(self.topic_depth))
        rospy.loginfo("subscribed to"+" "+str(self.topic_image))

        rospy.Subscriber(self.topic_depth, Image, self.callback_image_depth)
        rospy.Subscriber(self.topic_image, Image, self.callback_image)

    def callback_image_depth(self,data):
        if len(data.data)<=0:
            rospy.logwarn("depth image from camera null")
        self.image_depth=data

    def callback_image(self,data):
        if len(data.data)<=0:
            rospy.logwarn("image rgb from camera null")
        self.image=data

    def handle_bbox_req(self,req):
        """
        Get the message with the bounding boxes coordinates.
        Extract the data from the message and give the pixel coordinates (height,width).
        Call the get_center_coordinates function
        """
        
        
        model_name = req.model_name
        classes = req.classes
        if len(req.depth.data)>0:
            rospy.loginfo("service called without depth_image")
            self.depth_msg=req.depth
        else:
            self.depth_msg =  self.image_depth
        


        if len(req.image.data)>0 :
            rospy.loginfo("service called without rgb_image")
            self.image_msg=req.image
        else:
            self.image_msg =  self.image
        # rospy.loginfo("en attente du cul")

        rospy.wait_for_service('yolov8_on_unique_frame')
        try:
            # rospy.loginfo("service du cul activé")
            yolov8_cli=rospy.ServiceProxy('yolov8_on_unique_frame', Yolov8)
            resp=yolov8_cli(model_name,classes,self.image_msg).boxes_pose
            # rospy.loginfo(len(resp))
            list_score=[]
            list_label=[]
            list_height=[]
            list_width=[]
            list_id=[]
            for bbox in resp:

                # rospy.loginfo("les couilles")
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = self.depth_msg.header.frame_id # Frame de référence global
                transform.child_frame_id = "OBJ_" + str(bbox.bbox_class) + '_link' # Frame de l'objet détecté 
                transform.transform.translation.x = [bbox.ymax-(bbox.ymax-bbox.ymin)/2,bbox.xmax-(bbox.xmax-bbox.xmin)/2]
                transform.transform.translation.y = bbox.xmax-(bbox.xmax-bbox.xmin)/2
                transform.transform.translation.z = bbox.ymax-(bbox.ymax-bbox.ymin)/2
                transform.transform.rotation.x = 0.0
                transform.transform.rotation.y = 0.0
                transform.transform.rotation.z = 0.0
                transform.transform.rotation.w = 1.0
                list_score.append(bbox.probability)
                list_label.append(bbox.bbox_class)
                center=[bbox.ymax-(bbox.ymax-bbox.ymin)/2,bbox.xmax-(bbox.xmax-bbox.xmin)/2]
                list_height.append(center[0])
                list_width.append(center[1])
                list_id.append(bbox.ID)
                

            returned_boxes=boxes3D_pose()
            returned_boxes=[]
            # rospy.loginfo(len(returned_boxes))

            for bbox in resp:
                # rospy.loginfo("labite")
                box=Box3D_pose()
                box.ID=bbox.ID
                box.bbox_class=bbox.bbox_class
                box.probability=bbox.probability
                box.xmin=bbox.xmin
                box.ymin=bbox.ymin
                box.xmax=bbox.xmax
                box.ymax=bbox.ymax
                # rospy.loginfo(center)
                box.centerz=self.get_centers_coordinates(center)
                # rospy.loginfo(box.centerz)
                
                box.skeleton=bbox.skeleton
                # rospy.loginfo(box)
                returned_boxes.append(box)
                rospy.loginfo(str(returned_boxes))           
            rospy.loginfo(returned_boxes)
            return(boxes3DResponse(returned_boxes))



        except:
            pass

    def get_centers_coordinates(self,center):
        """
        Get a data array with the label of each object and the pixel "coordinates" of its center,
        For each center, the program checks in the Point Cloud data to get XYZ coordinates
        related to the kinect system.
        Will publish a custom message with a table containing each point XYZ with its label
        """
        

        depth_array=self.bridge.imgmsg_to_cv2(self.depth_msg,"16UC1")
        rospy.loginfo(depth_array.shape)
        depth=depth_array[int(center[0])][int(center[1])]
        rospy.loginfo(depth)
        if depth==0:
            depth=-1
        return depth
    
if __name__=="__main__":
    if __name__ == '__main__':
        a=GetCenterCoordinates()
        while not rospy.is_shutdown():
            a.rate.sleep()