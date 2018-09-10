#! /usr/bin/python2.7

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import pyttsx


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

    return faces, rects


def detect_face(img):
    '''get a face (the largest) from image'''
    faces, rects = detect_faces(img)

    if len(faces) > 0:
        for i in range(len(faces)):
            draw_rectangle(img, rects[i])
        else:
            draw_text(img, "No face", 100, 100)

        cv2.namedWindow("camera", 0)
        cv2.imshow("camera", img)
        cv2.waitKey(10)

        if len(faces) == 1:
            return faces[0]
        else:
            max_area = 0
            f = faces[0]
            for e in enumerate(rects):
                (_, _, w, h) = e[1]
                area = w * h
                if area > max_area:
                    f = faces[e[0]]

            cv2.namedWindow("face", 0)
            cv2.imshow("face", f)
            cv2.waitKey(10)

            return f
    else:
        return None


def say(string):
    speech = pyttsx.init()
    speech.say(string)
    speech.runAndWait()


class Run:
    def __init__(self):
        self.data = {}
        self.MAX_FACES = 10

        self.command = ''
        self.name = ''
        self.face_recognizer = None
        self.bridge = CvBridge()  # convert img to cv2

        rospy.init_node('project_main')
        self.cmd_pub = rospy.Publisher('/cmd', String, queue_size=10)
        rospy.Subscriber("/camera/color/image_raw",
                         Image, self.callback_camera)

        say('I am ready.')
        rospy.loginfo("project started")

    def meet(self, name):
        self.command = 'meet'
        self.name = name
        if self.data.get(name) is None:
            self.data[name] = []

    def train(self):
        self.command = 'train'

    def find(self, name):
        self.command = 'find'
        self.name = name

    def callback_camera(self, cam_data):
        try:
            img = self.bridge.imgmsg_to_cv2(cam_data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.command == 'meet':
            if len(self.data.get(self.name)) >= self.MAX_FACES:
                say('Hello ' + self.name + '. How are you?')
                self.command = 'done meet'
                rospy.loginfo("done meet " + self.name)

            else:
                face = detect_face(img)
                if face is not None:
                    self.data[self.name].append(face)
                    rospy.loginfo("added " + self.name + "'s face " +
                                  str(len(self.data.get(self.name))))

        elif self.command == 'train':
            if len(self.data.values()) > 0:
                # create our LBPH face recognizer
                self.face_recognizer = cv2.face.LBPHFaceRecognizer_create()
                #face_recognizer = cv2.face.EigenFaceRecognizer_create()
                #face_recognizer = cv2.face.FisherFaceRecognizer_create()

                faces = []
                labels = []
                for e in enumerate(self.data.values()):
                    for f in e[1]:
                        faces.append(f)
                        labels.append(e[0])

                # train our face recognizer of our training faces
                self.face_recognizer.train(faces, np.array(labels))

                self.command = 'done train'
                rospy.loginfo('done train')

        elif self.command == 'find':
            TH = 50

            names = self.data.keys()
            faces, _ = detect_faces(img)

            if len(faces) > 0:
                for f in faces:
                    # predict the image using our face recognizer
                    label, confidence = self.face_recognizer.predict(f)
                    print('name', names[label], 'confidence', confidence)
                    if confidence > TH:
                        if names[label] == self.name:
                            say('This is ' + self.name + '. I found you!')

                            self.command = 'done find'
                            rospy.loginfo("Found " + self.name)


def send_cmd_loop(app):
    while True:
        command = raw_input()  # Wait for a user to print something.

        if command == '\n':
            break
        elif command == '':
            break
        elif command == 'train':
            app.train()
        else:
            cmd = command.split(' ')
            if len(cmd) == 2:
                if cmd[0] == 'meet':
                    app.meet(cmd[1])
                elif cmd[0] == 'find':
                    app.find(cmd[1])

        rospy.Rate(10).sleep()


if __name__ == '__main__':
    app = Run()
    send_cmd_loop(app)
    cv2.destroyAllWindows()
