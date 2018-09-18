#! /usr/bin/python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes


def draw_rectangle(img, rect):
    (x, y, w, h) = rect
    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)


def draw_text(img, text, x, y):
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2)


def detect_faces(img):
    '''detect faces using OpenCV'''
    # convert the test image to gray image as opencv face detector expects gray images
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # load OpenCV face detector, I am using LBP which is fast
    # there is also a more accurate but slow Haar classifier
    #face_cascade = cv2.CascadeClassifier('opencv-files/lbpcascade_frontalface.xml')
    #face_cascade = cv2.CascadeClassifier("opencv-files/haarcascade_frontalface_default.xml")
    face_cascade = cv2.CascadeClassifier(
        "opencv-files/haarcascade_frontalface_alt.xml")

    # let's detect multiscale (some images may be closer to camera than others) images
    # result is a list of faces
    rects = face_cascade.detectMultiScale(
        gray, scaleFactor=1.3, minNeighbors=5)

    # crop faces
    faces = []
    if (len(rects) != 0):
        for rect in rects:
            (x, y, w, h) = rect
            faces.append(gray[y:y+w, x:x+h])

        # Show img (for debug)
        for i in range(len(faces)):
            draw_rectangle(img, rects[i])
        cv2.namedWindow("camera")
        cv2.imshow("camera", img)
        cv2.waitKey(100)

    return faces, rects


def detect_face(img):
    '''get a face (the largest) from image'''
    faces, rects = detect_faces(img)

    if len(faces) > 0:
        f = faces[0]

        if len(faces) > 1:
            max_area = 0
            for k in range(len(rects)):
                (_, _, w, h) = rects[k]
                area = w * h
                if area > max_area:
                    f = faces[k]

        # Show face (for debug)
        cv2.namedWindow("face")
        cv2.imshow("face", f)
        cv2.waitKey(100)

        return f
    else:
        return None


class Run:
    def __init__(self):
        self.data = {}
        self.MAX_FACES = 10

        self.command = ''
        self.name = ''
        self.face_recognizer = None
        self.bridge = CvBridge()  # convert img to cv2
        self.boxes = None

        rospy.init_node('project_main')
        self.voice_pub = rospy.Publisher('/voice', String, queue_size=10)
        rospy.Subscriber("/camera/color/image_raw",
                         Image, self.callback_camera)
        rospy.Subscriber("/darknet_ros/detection_image",
                         Image, self.callback_detection_image)
        rospy.Subscriber("/darknet_ros/bounding_boxes",
                         BoundingBoxes, self.callback_boxes)

        self.voice('I am ready.')
        rospy.loginfo("project started")

    def voice(self, text):
        import pyttsx
        speech = pyttsx.init()
        speech.say(text)
        speech.runAndWait()
        self.voice_pub.publish(text)

    def taskdone(self, text):
        self.command = ''  # reset to empty
        self.voice(text)
        rospy.loginfo(text)

    def callback_boxes(self, data):
        '''
        Header header
        Header image_header
        BoundingBox[] bounding_boxes

        string Class
        float64 probability
        int64 xmin
        int64 ymin
        int64 xmax
        int64 ymax
        '''
        self.boxes = data.bounding_boxes

    def callback_detection_image(self, data):
        try: 
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # get image height, width
            #(h, w) = img0.shape[:2]
            # calculate the center of the image
            #center = (w / 2, h / 2)
            # rotate
            #M = cv2.getRotationMatrix2D(center, 90, 1.0)
            #img = cv2.warpAffine(img0, M, (h, w))
        except CvBridgeError as e:
            print(e)
            
        if self.boxes: 
            for b in self.boxes:
                if b.Class == 'person':
                    # Show person (for debug)
                    cv2.namedWindow("person")
                    cv2.imshow("person", img[b.ymin:b.ymax, b.xmin:b.xmax])
                    cv2.waitKey(100)

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

        if self.command == 'meet':
            if self.data.get(self.name) is None:
                self.data[self.name] = []

            elif len(self.data.get(self.name)) >= self.MAX_FACES:
                self.taskdone('Hello ' + self.name + '. How are you?')

            else:
                face = detect_face(img)
                if face is not None:
                    self.data[self.name].append(face)
                    rospy.loginfo("added " + self.name + "'s face " +
                                  str(len(self.data.get(self.name))))

        elif self.command == 'train':
            people = self.data.values()
            if len(people) > 0:
                # create our LBPH face recognizer
                self.face_recognizer = cv2.face.LBPHFaceRecognizer_create()
                #face_recognizer = cv2.face.EigenFaceRecognizer_create()
                #face_recognizer = cv2.face.FisherFaceRecognizer_create()

                # prepare data for training
                faces = []
                labels = []
                for k in range(len(people)):
                    for f in people[k]:
                        faces.append(f)
                        labels.append(k)

                # train our face recognizer of our training faces
                self.face_recognizer.train(faces, np.array(labels))
                self.taskdone('Finished training.')

        elif self.command == 'find':
            TH = 80

            names = self.data.keys()
            faces, _ = detect_faces(img)

            if len(faces) > 0:
                for f in faces:
                    # predict the image using our face recognizer
                    label, confidence = self.face_recognizer.predict(f)
                    print('name', names[label], 'confidence', confidence)

                    # Show face (for debug)
                    cv2.namedWindow("face")
                    cv2.imshow("face", f)
                    cv2.waitKey(100)

                    if confidence < TH:
                        if names[label] == self.name:
                            self.taskdone('This is ' + self.name +
                                          '. I found you!')


def send_cmd_loop(app):
    while True:
        command = raw_input()  # Wait for a user to print something.
        cmd = command.split(' ')

        if command == '\n':
            break
        elif command == '':
            break
        elif len(cmd) == 2:
            app.command = cmd[0]
            app.name = cmd[1]
        else:
            app.command = command

        rospy.Rate(10).sleep()


if __name__ == '__main__':
    app = Run()
    send_cmd_loop(app)
    cv2.destroyAllWindows()
