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
from geometry_msgs.msg import PoseStamped, TransformStamped
from math import pi,sin
# from darknet_ros_msgs.msg import BoundingBoxes
from tf.msg import tfMessage
from yolov8_ros_msgs.srv import Yolov8
from bbox3D_msgs.srv import bbox3D_srv, bbox3D_srvResponse
# from Boxes.msg import boxes
from bbox3D_msgs.msg import Box3D_pose
# from tf_broadcaster.msg import DetectionCoordinates, PointCoordinates
from nav_msgs.msg import Odometry


class GetCenterCoordinates(object):
    def __init__(self):
        """
        Create an instance of GetCenterCoordinates Class.
        Setup the ROS node and the parameters used in the code
        """
        rospy.init_node("calcul_pose_boxes",anonymous=True)
        self.bridge = CvBridge()
        self.depth_msg=Image()
        self.rate=rospy.Rate(30)
        self.center_coordinates=None
        self.topic_image= rospy.get_param("/topic_image","/kinect2/hd/image_color")
        self.topic_depth= rospy.get_param("/topic_depth","/kinect2/hd/image_depth_rect")

        self.boxes_src = rospy.Service('boxes_3d_service', bbox3D_srv, self.handle_bbox_req)
        rospy.loginfo("subscribed to"+" "+str(self.topic_depth))
        rospy.loginfo("subscribed to"+" "+str(self.topic_image))


        rospy.Subscriber(self.topic_depth, Image, self.callback_image_depth)
        rospy.Subscriber(self.topic_image, Image, self.callback_image)

        self.tf_pub = rospy.Publisher("/tf", tfMessage, queue_size=10)

    def callback_image_depth(self,data):
        if len(data.data)<=0:
            rospy.logwarn("[bbox3D_node] depth image from camera null")
        self.image_depth=data

    def callback_image(self,data):
        if len(data.data)<=0:
            rospy.logwarn("[bbox3D_node] image rgb from camera null")
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
            rospy.loginfo("[bbox3D_node] service called without depth_image")
            self.depth_msg=req.depth
        else:
            self.depth_msg =  self.image_depth
        


        if len(req.image.data)>0 :
            rospy.loginfo("[bbox3D_node] service called without rgb_image")
            self.image_msg=req.image
        else:
            self.image_msg =  self.image

        rospy.wait_for_service('yolov8_on_unique_frame')
        try:
            yolov8_cli=rospy.ServiceProxy('yolov8_on_unique_frame', Yolov8)
            resp=yolov8_cli(model_name,classes,self.image_msg).boxes
            # rospy.loginfo(len(resp))
                

            returned_boxes=[]
            # rospy.loginfo(len(returned_boxes))

            for bbox in resp.boxes:
                box=Box3D_pose()
                box.ID=bbox.ID
                box.bbox_class=bbox.bbox_class
                box.probability=bbox.probability
                box.xmin=bbox.xmin
                box.ymin=bbox.ymin
                box.xmax=bbox.xmax
                box.ymax=bbox.ymax
                centerz=[bbox.ymax-(bbox.ymax-bbox.ymin)/2,bbox.xmax-(bbox.xmax-bbox.xmin)/2]
                centery=bbox.ymax-(bbox.ymax-bbox.ymin)/2
                centerx=bbox.xmax-(bbox.xmax-bbox.xmin)/2
                box.pose=self.get_centers_coordinates(centerx,centery,centerz,bbox.bbox_class)
                # rospy.loginfo(center)
                # rospy.loginfo(box.centerz)
                
                box.skeleton=bbox.skeleton
                # rospy.loginfo(box)
                returned_boxes.append(box)
                rospy.loginfo("[bbox3D_node] Pose sent")
                # rospy.loginfo(str(returned_boxes))           
            # rospy.loginfo(returned_boxes)
            return(bbox3D_srvResponse(returned_boxes))

        except:
            pass


    def get_centers_coordinates(self,centerx,centery,centerz,label):
        """
        Get a data array with the label of each object and the pixel "coordinates" of its center,
        For each center, the program checks in the Point Cloud data to get XYZ coordinates
        related to the kinect system.
        Will publish a custom message with a table containing each point XYZ with its label
        """
        pose=PoseStamped()
        depth_array=self.bridge.imgmsg_to_cv2(self.depth_msg,"16UC1")
        # rospy.loginfo(depth_array.shape)
        depth=depth_array[int(centerz[0])][int(centerz[1])]
        # rospy.loginfo(depth)
        if depth==0:
            depth=-1
        if depth==-1:
            # rospy.loginfo(pose)
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id=self.image_msg.header.frame_id
            return pose
        else :
            depth=depth/1000.0
            x_real,y_real=self.getRealXY(centerx,centery,depth,self.image_msg.width,self.image_msg.height)
            pose.header.stamp = rospy.Time.now()
            rospy.loginfo(self.image_msg.header.frame_id)
            pose.header.frame_id = self.image_msg.header.frame_id  # Frame de référence global
            # pose.child_frame_id = "OBJ_" + label + '_link' # Frame de l'objet détecté 
            pose.pose.position.x = depth
            pose.pose.position.y = x_real
            pose.pose.position.z = y_real
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            
            # rospy.loginfo(pose)
            tf_msg = tfMessage()
            stamped_msg = TransformStamped()
            stamped_msg.header = self.image_msg.header
            stamped_msg.child_frame_id = "sac"
            stamped_msg.transform.translation.x = pose.pose.position.x
            stamped_msg.transform.translation.y = pose.pose.position.y
            stamped_msg.transform.translation.z = pose.pose.position.z
            stamped_msg.transform.rotation.x = pose.pose.orientation.x
            stamped_msg.transform.rotation.y = pose.pose.orientation.y
            stamped_msg.transform.rotation.z = pose.pose.orientation.z
            stamped_msg.transform.rotation.w = pose.pose.orientation.w
            tf_msg.transforms.append(stamped_msg)
            self.tf_pub.publish(tf_msg)
            return  pose

    
    def getRealXY(self, x_ref, y_ref, distance, img_w=640, img_h=480, HFovDeg=70, VFovDeg=60):
        
        HFov = HFovDeg * pi / 180.0  # Horizontal field of view of the RealSense D455
        VFov = VFovDeg * pi / 180.0
        #Phi = (HFov / 2.0) * ( (2*neck_x)/self.image_w + 1)  #Angle from the center of the camera to neck_x
        PhiX = (HFov / 2.0) *  (x_ref - img_w/2) / (img_w/2) #Angle from the center of the camera to neck_x
        PhiY = (VFov / 2.0) *  (y_ref - img_h/2) / (img_h/2)
        return (    distance * sin(PhiX)  ,     distance * sin(PhiY)   )

    
if __name__=="__main__":
    if __name__ == '__main__':
        a=GetCenterCoordinates()
        while not rospy.is_shutdown():
            a.rate.sleep()