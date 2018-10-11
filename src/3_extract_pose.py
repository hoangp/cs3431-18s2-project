#!/usr/bin/env python
import rospy
import numpy as np
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

sys.path.append('/home/sassbot/openpose/build/python');

# Parameters for OpenPose. Take a look at C++ OpenPose example for meaning of components. Ensure all below are filled
try:
    from openpose import OpenPose
except:
    raise Exception('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')


class Run:
    def __init__(self):
        rospy.init_node('show_img')

        params = dict()
        params["logging_level"] = 3
        params["output_resolution"] = "-1x-1"
        params["net_resolution"] = "-1x368"
        params["model_pose"] = "BODY_25"
        params["alpha_pose"] = 0.6
        params["scale_gap"] = 0.3
        params["scale_number"] = 1
        params["render_threshold"] = 0.05
        # If GPU version is built, and multiple GPUs are available, set the ID here
        params["num_gpu_start"] = 0
        params["disable_blending"] = False
        # Ensure you point to the correct path where models are located
        params["default_model_folder"] = "/home/sassbot/openpose/models/"
        # Construct OpenPose object allocates GPU memory
        self.op = OpenPose(params)

        self.bridge = CvBridge()  # convert img to cv2
        rospy.Subscriber("/camera/rgb/image_color",
                        Image, self.callback_camera)



        print "Ready"

    def callback_camera(self, cam_data):
        try: 
            img = self.bridge.imgmsg_to_cv2(cam_data, "bgr8")
            # get image height, width
            #(h, w) = img0.shape[:2]
            # calculate the center of the image
            #center = (w / 2, h / 2)
            # rotate
            #M = cv2.getRotationMatrix2D(center, 90, 1.0)
            #img = cv2.warpAffine(img0, M, (h, w))
        except CvBridgeError as e:
            print(e)

        cv2.imshow("input", img)
        cv2.waitKey(15)

        self.show_img(img)


    def show_img(self, img):
        keypoints, output_image = self.op.forward(img, True)

        cv2.imshow("output", output_image)
        cv2.waitKey(15)


if __name__ == "__main__":
    app = Run()
    img = cv2.imread("../media/COCO_val2014_000000000192.jpg")
    app.show_img(img)
    rospy.spin()
    cv2.destroyAllWindows()