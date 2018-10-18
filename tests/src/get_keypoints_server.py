#!/usr/bin/env python
import rospy
import numpy as np
from person_recognition.srv import *
import sys
import cv2

sys.path.append('/home/sassbot/openpose/build/python');

# Parameters for OpenPose. Take a look at C++ OpenPose example for meaning of components. Ensure all below are filled
try:
    from openpose import OpenPose
except:
    raise Exception('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')


class GetKeyPointsServer:
    def __init__(self):
        rospy.init_node('get_keypoints_server')
        rospy.Service('get_keypoints', GetKeyPoints, self.handle_get_keypoints) 

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

        print "Ready to get_keypoints."

    def handle_get_keypoints(self, req):
        print("received req", len(req.imgdata))

        #img = np.reshape( np.array(req.imgdata, dtype="uint8") , (480,640,3) )

        img = cv2.imread("../media/COCO_val2014_000000000192.jpg")

        keypoints, output_image = self.op.forward(img, True)

        # Display the image
        cv2.imshow("output", output_image)
        cv2.waitKey(15)

        keypoints = req.imgdata
        print("return keypoints", keypoints)

        return GetKeyPointsResponse(keypoints)


if __name__ == "__main__":
    app = GetKeyPointsServer()
    rospy.spin()
    cv2.destroyAllWindows()
