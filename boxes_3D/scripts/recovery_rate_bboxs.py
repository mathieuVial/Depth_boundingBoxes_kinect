#!/usr/bin/env python3

import rospy
# from sensor_msgs.msg import PointCloud2
import os
# from sensor_msgs import convertPointCloud2ToPointCloud
# from sensor_msgs.point_cloud2 import read_points
import numpy
import cv2
from cv_bridge import CvBridge# import ros_numpy as rosnp
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from math import pi,sin
# from darknet_ros_msgs.msg import BoundingBoxes
from tf.msg import tfMessage
from boxes_3D.srv import boxes3D, boxes3DResponse
import scp

from boxes_3D.msg import Box3D_pose


class Get_Recovery_rate(object):
    def __init__(self):
        """
        Create an instance of GetCenterCoordinates Class.
        Setup the ROS node and the parameters used in the code
        """
        rospy.init_node("box_to_carry",anonymous=True)
        self.bridge = CvBridge()
        self.depth_msg=Image()
        self.rate=rospy.Rate(30)
        self.center_coordinates=None
        self.boxes_src = rospy.Service('box_to_carry_service', boxes3D, self.myServiceCallback)
        self.image = Image()
        self.bridge=CvBridge()
        self.list_coord_case=[]
        self.cv2_pub = rospy.Publisher("suitcase", Image, queue_size=10)
        self.topic_image = rospy.Subscriber("/kinect2/hd/image_color", Image, self.image_callback, queue_size=1)

        rospy.spin()
    def image_callback(self,data):
        self.image=data


    def myServiceCallback(self,req):


            rospy.wait_for_service('boxes_3d_service')
        

        # service call boxes_3d

            try:

                rospy.loginfo("Attente du service boxes 3D")

                get_boxes_espace = rospy.ServiceProxy('boxes_3d_service',boxes3D)
                bbox_espace_suitcases = get_boxes_espace("",[28],Image(),Image()).boxes3D_pose
                bbox_espace_people = get_boxes_espace("yolov8m-pose.pt",[0],Image(),Image()).boxes3D_pose
                self.list_coord_case=[]

                
                air_max=0
                largest_bbox=Box3D_pose()
                rospy.loginfo("test")
                for human in bbox_espace_people:
                    air=abs((human.xmin-human.xmax))*abs((human.ymin-human.ymax))
                    if air>air_max and len(human.skeleton)>0:
                        air_max=air
                        largest_bbox = human
                flag_arm=0
                arm = []
                delta_l=abs(largest_bbox.skeleton[7].x-largest_bbox.skeleton[9].x)
                delta_r=abs(largest_bbox.skeleton[8].x-largest_bbox.skeleton[10].x)
                if delta_r>delta_l:
                    flag_arm=1
                    arm = [largest_bbox.skeleton[8],largest_bbox.skeleton[10]]
                else :
                    arm = [largest_bbox.skeleton[7],largest_bbox.skeleton[9]]
                rospy.loginfo(flag_arm)

                if flag_arm:
                    if largest_bbox.skeleton[10].x<largest_bbox.skeleton[8].x:
                        #x <
                        dir="inf"
                    else:
                        #x >
                        dir="sup"
                else :
                    if largest_bbox.skeleton[9].x>largest_bbox.skeleton[7].x:
                        #x >
                        dir="sup"
                    else:
                        #x <
                        dir="inf"

                delta_box_min = -1
                box_to_carry = Box3D_pose()
                for box in bbox_espace_suitcases:
                    
                    center=[(box.xmin+box.xmax)/2,(box.ymin+box.ymax)/2]

                    if dir == "sup":
                        rospy.loginfo(dir)
                        if (center[0]) > arm[1].x:
                            if delta_box_min==-1 or abs(arm[1].x-center[0])<delta_box_min:
                                intersection_x, intersection_y=lineLineIntersection(arm[0], arm[1],center,(center[0],center[1]+10))
                                if intersection_y<box.ymax and intersection_y>box.ymin:
                                    delta_box_min = abs(arm[1].x-center[0])
                                    box_to_carry=box


                    else:
                        rospy.loginfo(dir)
                        if center[0] < arm[1].x:
                            if delta_box_min==-1 or abs(arm[1].x-center[0])<delta_box_min:
                                intersection_x, intersection_y=lineLineIntersection(arm[0], arm[1],center,(center[0],center[1]+10) )
                                if intersection_y<box.ymax and intersection_y>box.ymin:
                                    delta_box_min = abs(arm[1].x-center[0])
                                    box_to_carry=box
                rospy.loginfo(box_to_carry)

                

                min_pt = (int(box_to_carry.xmin),int(box_to_carry.ymin))
                max_pt =  (int(box_to_carry.xmax),int(box_to_carry.ymax))
                cv_image = self.bridge.imgmsg_to_cv2(self.image)
                cv2.rectangle(cv_image, min_pt, max_pt, color = (0,255,0), thickness=10)
            
                cv2.imwrite('/tmp/image_luggage.jpg', cv_image)
                self.cv2_pub.publish((self.bridge.cv2_to_imgmsg(cv_image,
                                                               encoding=self.image.encoding)))
                client=scp.client(host="10.68.0.1",user="pal",password="")
                client.transfert('/tmp/suitcase.jpg','/home/robocup/Bureau/robocup_2023/ros_ws/src/ros_hri_manager/public_data/hri/suitcase.jpg')
                return(boxes3DResponse([box_to_carry]))



                    
                    #bras droit
                    
                    #bras gauche
                
                    

            except rospy.ServiceException as e:

                print("erreur d'appel du service")
    

def lineLineIntersection(A, B, C, D):
    
    # Line AB represented as a1x + b1y = c1
    a1 = B.y - A.y
    b1 = A.x - B.x
    c1 = a1*(A.x) + b1*(A.y)
 
    # Line CD represented as a2x + b2y = c2
    a2 = D[1] - C[1]
    b2 = C[0]- D[0]
    c2 = a2*(C[0]) + b2*(C[1])
 
    determinant = a1*b2 - a2*b1
 
    if (determinant == 0):
        # The lines are parallel. This is simplified
        # by returning a pair of FLT_MAX
        return None
    else:
        x = (b2*c1 - b1*c2)/determinant
        y = (a1*c2 - a2*c1)/determinant
        return x, y

def isect_line_plane_v3(p0, p1, p_co, p_no, epsilon=1e-6):
    """
    p0, p1: Define the line.
    p_co, p_no: define the plane:
        p_co Is a point on the plane (plane coordinate).
        p_no Is a normal vector defining the plane direction;
             (does not need to be normalized).

    Return a Vector or None (when the intersection can't be found).
    """

    u = sub_v3v3(p1, p0)
    dot = dot_v3v3(p_no, u)

    if abs(dot) > epsilon:
        # The factor of the point between p0 -> p1 (0 - 1)
        # if 'fac' is between (0 - 1) the point intersects with the segment.
        # Otherwise:
        #  < 0.0: behind p0.
        #  > 1.0: infront of p1.
        w = sub_v3v3(p0, p_co)
        fac = -dot_v3v3(p_no, w) / dot
        u = mul_v3_fl(u, fac)
        return add_v3v3(p0, u)

    # The segment is parallel to plane.
    return None

# ----------------------
# generic math functions

def add_v3v3(v0, v1):
    return (
        v0[0] + v1[0],
        v0[1] + v1[1],
        v0[2] + v1[2],
    )


def sub_v3v3(v0, v1):
    return (
        v0[0] - v1[0],
        v0[1] - v1[1],
        v0[2] - v1[2],
    )


def dot_v3v3(v0, v1):
    return (
        (v0[0] * v1[0]) +
        (v0[1] * v1[1]) +
        (v0[2] * v1[2])
    )


def len_squared_v3(v0):
    return dot_v3v3(v0, v0)


def mul_v3_fl(v0, f):
    return (
        v0[0] * f,
        v0[1] * f,
        v0[2] * f,
    )

if __name__ == '__main__':
    a=Get_Recovery_rate()
    
    
