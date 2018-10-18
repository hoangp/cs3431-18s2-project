import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import random
import numpy as np

# functions to test pictures
def draw_rectangle(img, rect):
    (x, y, w, h) = rect
    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

rospy.init_node('test_display')
crop_pub = rospy.Publisher('pr/crop', Image, queue_size=10)

while 1:
    # Read new image
    img = cv2.imread("../media/COCO_val2014_000000000192.jpg")

    bridge = CvBridge()

    y1 = random.randint(0,200)
    y2 = y1 + random.randint(100,200)

    x1 = random.randint(0,300)
    x2 = x1 + random.randint(100,300)

    # crop = img[y1:y2, x1:x2]
    
    rect = (x1,y1,x2-x1,y2-y1)
    draw_rectangle(img, rect)

    cam = bridge.cv2_to_imgmsg(img,encoding="bgr8")

    cv2.imshow("img", img)
    cv2.waitKey(15)

    crop_pub.publish(cam)
