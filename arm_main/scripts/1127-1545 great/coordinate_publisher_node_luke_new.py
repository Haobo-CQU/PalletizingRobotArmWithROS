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
import time

# 11.27 15:44测试完全正常，20次全部正确


def getContours(img):
    contours,hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # print(area)
        if area>20:
            # cv2.drawContours(imgContour, cnt, -1, (255, 0, 0), 3)
            peri = cv2.arcLength(cnt,True)
            #print(peri)
            approx = cv2.approxPolyDP(cnt,0.03*peri,True)
            objCor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)

            if objCor ==3: objectType ="Tri"
            elif objCor == 4:
                aspRatio = w/float(h)
                if aspRatio >0.98 and aspRatio <1.03: objectType= "Square"
                else:objectType="Rectangle"
            elif objCor>6:
                objectType= "Circles"
                circle1=[int(x+0.5*w),int(y+0.5*h)]
                global circle
                circle.append(circle1)
                # cv2.rectangle(imgContour, (x, y), (x + w, y + h), (255, 0, 0), 2)
                # cv2.putText(imgContour,objectType,
                #              (x+(w//2)-10,y+(h//2)-10),cv2.FONT_HERSHEY_COMPLEX,0.7,
                #              (255,0,0),2)

            else:objectType="None"

def CoordinateIdentification():
    # def __init__(self):    
        # 对比cv_bridge_test.py少了第12行
        # 创建cv_bridge，声明图像的发布者和订阅者
        # self.coordinate_info_pub = rospy.Publisher('/coordinate_topic', coordinate, queue_size=10)
    coordinate_info_pub = rospy.Publisher('/coordinate_topic', coordinate, queue_size=10)
    print("publisher has been created.")
        # self.bridge = CvBridge()
    # bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.coordinate_identification_callback)
        # print("subscriber has been created.")


    # def coordinate_identification_callback(self,data):
   
        
        # initialize list
    default_list = [420, 670, 100]
    list1 = list2 = list3 = list4 = list5 = list6 = list6 = list7 = list8 = list9 = default_list
    list10 = [list4, list6, list8, list5,list7, list9, list1, list2, list3]

        
        
                                        ###################
                                        #### GET IMAGE ####
                                        ###################

        ## Get image from camera with calibration
        # try:
        #     cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        #     # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        #     img = cv_image
        # except CvBridgeError as e:
        #     print e                            

        # Direct get image from camera
    cap = cv2.VideoCapture(0)
    time.sleep(2)
    while True:
        
        ret,frame = cap.read()
        if frame is None:
            continue
        else:
            ret, frame = cap.read()
            break
    # print(cap.get(3))
    # print(cap.get(4))
    
    # cv2.imshow("capture", frame)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    img = frame
    cap.release()

    ## Direct get image from local folder
    # img = cv2.imread(
    #     '/home/roshan/catkin_ws/src/ros_arm/arm_main/picture/WIN_20201010_14_54_21_Pro.jpg')
    # # cv2.imshow("raw_image",img)
    # cv2.waitKey(0)
    
    # rospy.loginfo("Get a image from camera by cv_bridge with calibration.")

    # self.recognize_loop(img)


                                ##############################
                                #### IMAGE IDENTIFICATION ####
                                ##############################    

        
        #######################  Initialize list  #######################

    try:
        # tantian顺序：绿圆/绿长方/绿方/蓝圆/蓝长方/蓝方/红圆/红长方/红方

        
        height, width = img.shape[:2]
        height = int(height)
        width = int(width)
        cv2.circle(img,center = (width // 2, height // 2) , radius = int(min(height, width) / 1.4) , color = 0, thickness = 230)
        
        # cv2.imshow("circle_cover",img)
        # cv2.waitKey(0)
        # imgContour = img.copy()
        gs_frame_all = cv2.GaussianBlur(img.copy(), (7, 7), 1)  # 高斯模糊
        hsv_all = cv2.cvtColor(gs_frame_all, cv2.COLOR_BGR2HSV)  # 转化成HSV图像
        color_dist_all = {'all': {'Lower': np.array([0, 41, 0]), 'Upper': np.array([179, 255, 255])}}
        # erode_hsv = cv2.erode(hsv, None, iterations=5)  # 腐蚀 粗的变细
        ##################################绿色########################################
        ball_color = 'all'
        inRange_hsv_all = cv2.inRange(hsv_all, color_dist_all[ball_color]['Lower'], color_dist_all[ball_color]['Upper'])
        kernel = np.ones((7, 7), np.uint8)
        opening_all = cv2.morphologyEx(inRange_hsv_all, cv2.MORPH_OPEN, kernel)
        # cv2.imshow('green', inRange_hsv_all)
        getContours(opening_all)
        # print(circle)

        # print("i come in, ok111?")
        # #######################  Cylinder  ############################
       
        # # careful! cv.CV_HOUGH_GRADIENT
        # print("i come in, ok112?")

        # print("i come in, ok113?")
        
        # circles = circles1[0, :, :]
        # print("i come in, ok01?")
        # circles = np.uint16(np.around(circles))

        for i in circle[:]:
            BGR = img[i[1], i[0]]
            # print(img[i[1], i[0]])#打印三个圆心的BGR值
            # print(i[0], i[1], i[2])  # 打印三个圆心的坐标
            if BGR[1] > BGR[0] and BGR[1] > BGR[2]:
                # print('Green Cylinder coordinate ', i[0],  i[1])
                list1 = [i[0], i[1], 0]
            if BGR[0] > BGR[1] and BGR[0] > BGR[2]:
                # print('Blue  Cylinder coordinate ', i[0],  i[1])
                list2 = [i[0], i[1], 0]
            if BGR[2] > BGR[0] and BGR[2] > BGR[1]:
                # print('Red   Cylinder coordinate ', i[0],  i[1])
                list3 = [i[0], i[1], 0]
            # cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
            cv2.circle(img, (i[0], i[1]), 2, (255, 255, 255), 70)
            # cv2.imshow("circle",img)
            # cv2.circle(img, (i[0], i[1]), 2, (255, 0, 255), 2)
        # cv2.imshow("sdasscylinder", img)
        # cv2.waitKey(0)
        # black circle
        # where=np.where
        # cir=np.zeros((480,640),np.uint8)
        # cv2.circle(cir,(320, 240),10, 255,-1)
        # img[where(cir==0)]=0

        # cv2.imshow("ddd",img)
        # cv2.waitKey(0)

        #######################  Rectangle  #######################
        # color_dist = {'red': {'Lower': np.array([168, 89, 5]), 'Upper': np.array([179, 255, 255])},
        #             'blue': {'Lower': np.array([35, 45, 5]), 'Upper': np.array([80, 255, 255])},
        #             'green': {'Lower': np.array([40, 39, 180]), 'Upper': np.array([163, 255, 255])},
        #             }# evening light without lamp
        color_dist = {'red': {'Lower': np.array([140, 57, 96]), 'Upper': np.array([179, 255, 215])},
                        'blue': {'Lower': np.array([0, 159, 173]), 'Upper': np.array([124, 234, 255])},
                        'green': {'Lower': np.array([30, 48, 95]), 'Upper': np.array([72, 255, 255])},
                        }# morning light without lamp
                                
        print("i come in, ok00?")
        gs_frame = cv2.GaussianBlur(img, (7, 7), 1)  # 高斯模糊
        hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)  # 转化成HSV图像
        # erode_hsv = cv2.erode(hsv, None, iterations=5)  # 腐蚀 粗的变细
        print("i come in, ok0?")
        ########  Green  ##########
        ball_color = 'green'
        print("i come in, ok1?")
        inRange_hsv = cv2.inRange(
            hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])
        #opening and closing
        kernel = np.ones((7, 7), np.uint8)
        opening = cv2.morphologyEx(inRange_hsv,cv2.MORPH_OPEN,kernel)
        # cv2.imshow("opening",opening)
        # cv2.waitKey(0)
        closing = cv2.morphologyEx(opening,cv2.MORPH_CLOSE,kernel)
        # cv2.imshow("closing",closing)
        # cv2.waitKey(0)
        # cv2.imshow("green",closing)
        # cv2.waitKey(0)
        cnts = cv2.findContours(
            closing.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        c = min(cnts, key=cv2.contourArea)
        b = max(cnts, key=cv2.contourArea)
        rect1 = cv2.minAreaRect(c)  # 正方形
        # print("Green Cube x y theta", rect1[0], rect1[2])
        list4 = [rect1[0][0], rect1[0][1], abs(rect1[2])]
        rect2 = cv2.minAreaRect(b)  # 长方形
        if rect2[1][0] > rect2[1][1]:
            # print("Green Cuboid x y theta", rect2[0], rect2[2])
            list5 = [rect2[0][0], rect2[0][1], abs(rect2[2])]
        else:
            # print("Green Cuboid x y theta", rect2[0], -(90 - rect2[2]))
            list5 = [rect2[0][0], rect2[0][1], abs(-(90 - rect2[2]))]
        # careful! cv.BoxPoints
        box1 = cv.BoxPoints(rect1)  # 正方形
        box2 = cv.BoxPoints(rect2)  # 长方形
        cv2.drawContours(img, [np.int0(box1)], -1, (0, 255, 255), 2)
        cv2.drawContours(img, [np.int0(box2)], -1, (0, 255, 255), 2)

        print("i come in, ok2?")
        ########  Blue  ########
        ball_color = 'blue'
        inRange_hsv = cv2.inRange(
            hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])
        #opening and closing
        kernel = np.ones((7, 7), np.uint8)
        opening = cv2.morphologyEx(inRange_hsv,cv2.MORPH_OPEN,kernel)
        # cv2.imshow("opening",opening)
        # cv2.waitKey(0)
        closing = cv2.morphologyEx(opening,cv2.MORPH_CLOSE,kernel)
        # cv2.imshow("closing",closing)
        # cv2.waitKey(0)
        # cv2.imshow("blue",closing)
        # cv2.waitKey(0)        
        cnts = cv2.findContours(
            closing.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        c = min(cnts, key=cv2.contourArea)
        b = max(cnts, key=cv2.contourArea)
        rect1 = cv2.minAreaRect(c)  # 正方形
        # print("Blue Cube x y theta", rect1[0], rect1[2])
        list6 = [rect1[0][0], rect1[0][1], abs(rect1[2])]
        rect2 = cv2.minAreaRect(b)  # 长方形
        if rect2[1][0] > rect2[1][1]:
            # print("Blue Cuboid x y theta", rect2[0], rect2[2])
            list7 = [rect2[0][0], rect2[0][1], abs(rect2[2])]
        else:
            # print("Blue Cuboid x y theta", rect2[0], -(90 - rect2[2]))
            list7 = [rect2[0][0], rect2[0][1], abs(-(90 - rect2[2]))]
        box1 = cv.BoxPoints(rect1)  # 正方形
        box2 = cv.BoxPoints(rect2)  # 长方形
        cv2.drawContours(img, [np.int0(box1)], -1, (0, 255, 255), 2)
        cv2.drawContours(img, [np.int0(box2)], -1, (0, 255, 255), 2)
        print("i come in, ok3?")
        ########  Red  ########
        ball_color = 'red'
        # hsvv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #print(hsvv[669,116])              
        inRange_hsv = cv2.inRange(
            hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])
                #opening and closing
        kernel = np.ones((7, 7), np.uint8)
        opening = cv2.morphologyEx(inRange_hsv,cv2.MORPH_OPEN,kernel)
        # cv2.imshow("opening",opening)
        # cv2.waitKey(0)
        closing = cv2.morphologyEx(opening,cv2.MORPH_CLOSE,kernel)
        # cv2.imshow("closing",closing)
        # cv2.waitKey(0)
        # cv2.imshow("red",closing)
        # cv2.waitKey(0)     
        cnts = cv2.findContours(
            closing.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        c = min(cnts, key=cv2.contourArea)
        b = max(cnts, key=cv2.contourArea)
        #cv2.imshow("ddddd",inRange_hsv)
        #cv2.waitKey(0)
        print("i come in, ok4?")
        rect1 = cv2.minAreaRect(c)  # 正方形
        # print("Red Cube x y theta", rect1[0], rect1[2])
        list8 = [rect1[0][0], rect1[0][1], abs(rect1[2])]
        rect2 = cv2.minAreaRect(b)  # 长方形
        if rect2[1][0] > rect2[1][1]:
            # print("Red Cuboid x y theta", rect2[0], rect2[2])
            list9 = [rect2[0][0], rect2[0][1], abs(rect2[2])]
        else:
            # print("Red Cuboid x y theta", rect2[0], -(90 - rect2[2]))
            list9 = [rect2[0][0], rect2[0][1],abs(-(90 - rect2[2]))]
        box1 = cv.BoxPoints(rect1)  # 正方形
        box2 = cv.BoxPoints(rect2)  # 长方形
        cv2.drawContours(img, [np.int0(box1)], -1, (0, 255, 255), 2)
        cv2.drawContours(img, [np.int0(box2)], -1, (0, 255, 255), 2)                
        # list1 绿圆 list2 蓝圆 list3 红圆
        # list4 绿方 list5 绿长方
        # list6 蓝方 list7 蓝长方
        # list8 红方 list9 红长方
        list10 = [list4, list6, list8, list5,list7, list9, list1, list2, list3]
        print("list10 is:")
        print(list10)
        # 画边框
        # cv2.drawContours(img, [np.int0(box1)], -1, (0, 255, 255), 2)
        # cv2.drawContours(img, [np.int0(box2)], -1, (0, 255, 255), 2)
        cv2.imshow('camera', img)
        cv2.waitKey(0)
        # cv2.destroyAllWindows()
    except Exception as e:
        print("Identification fault.")
        print(e.args)
        # print(str(e))
        # print(repr(e))
        pass




    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        coordinate_msg = coordinate()
        coordinate_msg.ObjDataArray = [0,0,0,0,0,0,0,0,0]
        j=0
        print("list10-2 is")
        print(list10)
        for i in list10:
            object_point = Point()
            object_point.x = i[0]
            object_point.y = i[1]
            object_point.z = i[2]
            coordinate_msg.ObjDataArray[j] = object_point
            j+=1

        coordinate_info_pub.publish(coordinate_msg)
        
            #rospy.spin()
        

        rate.sleep()



if __name__ == '__main__':
    try:
        # # initialize list
        # default_list = [180, 0, 100]
        # list1 = list2 = list3 = list4 = list5 = list6 = list6 = list7 = list8 = list9 = default_list
        # list10 = [list4, list6, list8, list5,list7, list9, list1, list2, list3]

        # initialize ros
        circle=[]
        rospy.init_node('coordinate_publisher_node')#anonymous=True
        rospy.loginfo("Starting coordinate_publisher_subscribed_camera node.")
        CoordinateIdentification()
        # print(list10[2])
        # rospy.spin()


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