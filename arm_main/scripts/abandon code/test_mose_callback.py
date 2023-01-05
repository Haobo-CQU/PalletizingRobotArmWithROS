import cv2
import numpy as np
import rospy
from arm_main.msg import coordinate
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import time

# mouse callback function
def draw_circle(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img,(x,y),100,(255,0,0),-1)



def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        cv2.circle(img, (x, y), 1, (255, 0, 0), thickness = -1)
        cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                    1.0, (0,255,0), thickness = 1)
        # print(x,y)
        global list4
        list4[0] = x
        list4[1] = y
        list4[2] = 90

        cv2.namedWindow("image",0)
        # cv2.resizeWindow("image", 1280, 960)
        cv2.imshow("image", img)

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





# Create a black image, a window and bind the function to window
cap = cv2.VideoCapture(0)
    # time.sleep(2)
while True:
    
    ret,frame = cap.read()
    if frame is None:
        continue
    else:
        ret, frame = cap.read()
        break
img = frame
cap.release()

rospy.init_node('test_mouse_node')
coordinate_info_pub = rospy.Publisher('/coordinate_topic', coordinate, queue_size=10)
default_list = [420, 670, 100]
list1 = list2 = list3 = list4 = list5 = list6 = list6 = list7 = list8 = list9 = default_list
list10 = [list4, list6, list8, list5,list7, list9, list1, list2, list3]







cv2.namedWindow("image",0)
cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
while(1):
    cv2.imshow("image", img)
    if cv2.waitKey(0)&0xFF==27:
        break
cv2.destroyAllWindows()




