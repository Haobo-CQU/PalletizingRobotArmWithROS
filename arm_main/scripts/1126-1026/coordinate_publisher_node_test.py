#!/usr/bin/env python
# coding=utf-8

# Created in 2020 by YuanHaobo  <yhb0521@126.com>
import rospy
from arm_main.msg import coordinate
from geometry_msgs.msg import Point
import cv2
import cv2.cv as cv #To avoid opencv's version and API_name conflicts
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import config

# this py file is modified in 2020 1027,1102,1111 by Yuan,Haobo.
# after [roslaunch usb_cam usb_cam_with_calibration.launch]
# it will subscribe /usb_cam/image_rect_color (published by usb_cam_node:usb_cam/raw --> image_proc node:)
#    to get a ROS image, which has been calibrated
# it will recognize the 9 objects' coordinates from the calibrated image
# it will publish /coordinate_topic with node_name:coordinate_publisher


class CoordinateIdentification:
    def __init__(self):    
        # 对比cv_bridge_test.py少了第12行
        # 创建cv_bridge，声明图像的发布者和订阅者
                # 在文件底部，发布前才创建
        self.coordinate_info_pub = rospy.Publisher('/coordinate_topic', coordinate, queue_size=10)
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        print("publisher has been created.")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color", Image, self.coordinate_identification_callback)
        print("subscriber has been created.")




    def coordinate_identification_callback(self,data):
        rate = rospy.Rate(1)
        # initialize list
        default_list = [699, 374, 90]
        list1 = list2 = list3 = list4 = list5 = list6 = list6 = list7 = list8 = list9 = default_list
        list10 = [list4, list6, list8, list5,list7, list9, list1, list2, list3]

        while not rospy.is_shutdown():
        
                                        ###################
                                        #### GET IMAGE ####
                                        ###################

            ## Get image from camera with calibration
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
                img = cv_image
                # cv2.imshow("cv_image",img)
                # cv2.waitKey(0)
            except CvBridgeError as e:
                print e                            

            ## Direct get image from camera
            # cap = cv2.VideoCapture(0)
            # ret, frame = cap.read()
            # # cv2.imshow("capture", frame)
            # # cv2.waitKey(0)
            # img = frame


            ## Direct get image from local folder
            # img = cv2.imread(
            #     '/home/roshan/catkin_ws/src/ros_arm/arm_main/picture/WIN_20201010_14_54_21_Pro.jpg')
            # # cv2.imshow("raw_image",img)
            # cv2.waitKey(0)
            
            rospy.loginfo("Get a image from camera by cv_bridge with calibration.")
            
                                    ##############################
                                    #### IMAGE IDENTIFICATION ####
                                    ##############################    

            
            try:
                totallist=[]
                where=np.where

                # img = cv2.imread("t2.png")
                img = cv2.resize(img, (320,160), interpolation = cv2.INTER_AREA)
                cv2.normalize(img, img, 0, 255, cv2.NORM_MINMAX)

                mask = np.zeros((160,320),np.uint8)
                cv2.circle(mask,(175,80),70,255,-1)
                cv2.rectangle(mask, (245,10), (310,150),0,20,4)
                img[where(mask == 0)] = 0
                
                # img = cv2.resize(img, (320,180), interpolation = cv2.INTER_AREA)


                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                print("ok1")
                for i in range(3):
                    #颜色分割
                    mask = cv2.inRange(hsv,np.array(config.get_const(str(i))['lower']),np.array(config.get_const(str(i))['upper']))
                    if i == 2:
                        mask2 = cv2.inRange(hsv,np.array(config.get_const(str(i))['lower2']),np.array(config.get_const(str(i))['upper2']))
                        mask[where(mask2!=0)] = 255
                    contours, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(img,contours,-1,(0,0,255),5)
                    # cv2.namedWindow("recog_image",cv2.WINDOW_NORMAL)                  
                    # cv2.imshow("recog_image",img)
                    print("ok2")
                    #查找圆
                    cirlist=[]
                    cir=0
                    for j in range(3):  
                        peri = cv2.arcLength(contours[j], True)
                        approx = cv2.approxPolyDP(contours[j], 0.03 * peri, True)

                        rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                        #cv2.drawContours(rgb,approx,-1,(0,0,255),5)
                        #cv2.imshow(str(i)+str(j),rgb)
                        if len(approx) > 6:
                            cir = j
                            break
                    #圆确定
                    (x,y),radius = cv2.minEnclosingCircle(contours[cir])
                    cirlist.append(round(x*4,4))
                    cirlist.append(round(y*4,4))
                    cirlist.append(90)
                    print("ok3")
                    #比较长方正方
                    comp=[]
                    rect=0
                    sq=0

                    for j in range(3):
                        if  j != cir:
                            box = cv2.minAreaRect(contours[j])
                            ratio = box[1][0]/box[1][1]
                            if ratio < 1:
                                ratio = 1/ratio
                            comp.append(j)
                            comp.append(ratio)

                    print("ok3.5")
                    if comp[1]>comp[3]:
                        rect=comp[0]
                        sq=comp[2]
                    else:
                        rect=comp[2]
                        sq=comp[0]
                    print("ok4")
                    #确定正方形
                    sqlist=[]
                    box = cv2.minAreaRect(contours[sq])
                    '''boxx = cv2.boxPoints(box)
                    boxx = np.int0(boxx)
                    cv2.drawContours(rgb,[boxx],-1,(0,0,255),2)'''
                    sqlist.append(round(box[0][0]*4,4))
                    sqlist.append(round(box[0][1]*4,4))
                    sqlist.append(round(-box[2],4))
                    print("ok5")
                    #确定长方形
                    rectlist=[]
                    box = cv2.minAreaRect(contours[rect])
                    '''boxx = cv2.boxPoints(box)
                    boxx = np.int0(boxx)
                    cv2.drawContours(rgb,[boxx],-1,(0,255,0),2)'''
                    rectlist.append(round(box[0][0]*4,4))
                    rectlist.append(round(box[0][1]*4,4))
                    if box[1][0]>box[1][1]:
                        rectlist.append(round(-box[2],4))
                    else:
                        rectlist.append(round(-box[2]+90,4))

                    totallist.append(sqlist)
                    totallist.append(rectlist)
                    totallist.append(cirlist)
                    print("ok6")

                # 再将opencv格式额数据转换成ros image格式的数据发布
                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
                except CvBridgeError as e:
                    print e


                list10=[totallist[0],totallist[3],totallist[6],totallist[1],totallist[4],totallist[7],totallist[2],totallist[5],totallist[8]]



                # self.coordinate_publisher()

                coordinate_msg = coordinate()
                coordinate_msg.ObjDataArray = [0,0,0,0,0,0,0,0,0]
                j=0
                print("list10 is")
                print(list10)
                for i in list10:
                    object_point = Point()
                    object_point.x = i[0]
                    object_point.y = i[1]
                    object_point.z = i[2]
                    coordinate_msg.ObjDataArray[j] = object_point
                    j+=1
                # print(coordinate_msg.ObjDataArray)
                # self.coordinate_info_pub = rospy.Publisher('/coordinate_topic', coordinate, queue_size=10)
                # print("publisher has been created.")
                self.coordinate_info_pub.publish(coordinate_msg)
                print("publishment has been done.")
            
                #rospy.spin()
            except Exception as e:
                print("Identification fault.")
                print(e.args)
                # print(str(e))
                # print(repr(e))
                pass

            rate.sleep()



if __name__ == '__main__':
    try:
        # # initialize list
        # default_list = [180, 0, 100]
        # list1 = list2 = list3 = list4 = list5 = list6 = list6 = list7 = list8 = list9 = default_list
        # list10 = [list4, list6, list8, list5,list7, list9, list1, list2, list3]

        # initialize ros
        rospy.init_node('coordinate_publisher_node')#anonymous=True
        rospy.loginfo("Starting coordinate_publisher_subscribed_camera node.")
        CoordinateIdentification()
        # print(list10[2])
        rospy.spin()


    except rospy.ROSInterruptException:
        print ("Shutting down coordinate_publisher_subscribed_camera node.")
        # cv2.destroyAllWindows()
        pass










                                    ##############################
                                    ####     Abandon Code     ####
                                    ##############################    

 #list10 = [list1, list2, list3, list4,list5, list6, list7, list8, list9]

#Abandon Code First generation
"""
def coordinate_publisher():                    
    # rospy.init_node('coordinate_publisher_subscribed_camera', anonymous=True)
    # has been put into main

    # coordinate_info_pub = rospy.Publisher('/coordinate_topic', coordinate, queue_size=10)
    # has been put into class CoordinateIdentification 's def __init__(self):

    # rate = rospy.Rate(2)
    # while not rospy.is_shutdown():
    #   ...
    #   rate.sleep()
    # has been remove to class CoordinateIdentification 's def coordinate_identification_callback(self,data):\

"""

# Abandon Code Second generation(2020.11.11 19:27)
"""
# def coordinate_publisher():
#     coordinate_msg = coordinate()
#     coordinate_msg.ObjDataArray = [0,0,0,0,0,0,0,0,0]
#     # coordinate_msg.ObjDataArray=copy.deepcopy(list10)
#     j=0
#     for i in list10:
#         object_point = Point()
#         object_point.x = i[0]
#         object_point.y = i[1]
#         object_point.z = i[2]
#         coordinate_msg.ObjDataArray[j] = object_point
#         j+=1
#     # print(coordinate_msg.ObjDataArray)
#     coordinate_info_pub = rospy.Publisher('/coordinate_topic', coordinate, queue_size=10)
#     print("publisher has been created.")
#     coordinate_info_pub.publish(coordinate_msg)

"""



"""Abandon Code

        # 再将opencv格式额数据转换成ros image格式的数据发布
        # try:
        #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # except CvBridgeError as e:
        #     print e




## Test Result


# ('Red   Cylinder coordinate ', 512, 286)
# ('Blue  Cylinder coordinate ', 698, 398)
# ('Green Cylinder coordinate ', 442, 462)
# ('Green Cube x y theta', (884.6441040039062, 452.00714111328125), -80.83765411376953)
# ('Green Cuboid x y theta', (724.470703125, 570.92626953125), -73.83549499511719)
# ('Blue Cube x y theta', (702.8731079101562, 245.6168670654297), -58.570438385009766)
# ('Blue Cuboid x y theta', (830.3216552734375, 231.14080810546875), -101.72511196136475)
# ('Red Cube x y theta', (592.5, 598.5), -45.0)
# ('Red Cuboid x y theta', (589.3314819335938, 548.2697143554688), -147.9946174621582)

# [[442, 462, 0], [698, 398, 0], [512, 286, 0], 
# [884.6441040039062, 452.00714111328125, -80.83765411376953], 
# [724.470703125, 570.92626953125,  73.83549499511719], 

# [702.8731079101562, 245.6168670654297, -58.570438385009766], 
# [830.3216552734375, 231.14080810546875, -101.72511196136475]

# [592.5, 598.5, -45.0], 
# [589.3314819335938, 548.2697143554688, -147.9946174621582]]


"""