#!/usr/bin/env python
import rospy
import numpy as np
from person_recognition.srv import *
import cv2

def get_keypoints_client(imgdata):
    rospy.wait_for_service('get_keypoints')
    try:
        get_keypoints = rospy.ServiceProxy('get_keypoints', GetKeyPoints)
        resp = get_keypoints(imgdata)
        return resp.keypoints
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    
    #imgdata = np.array([1,2,3,4])

    img = cv2.imread("../media/COCO_val2014_000000000192.jpg")

    print(img.shape)

    imgdata = np.ravel(img)

    imgdata = np.array([1,2,3,4])

    print (get_keypoints_client(imgdata))