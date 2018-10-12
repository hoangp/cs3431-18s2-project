#! /usr/bin/python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

POSE_BODY_25_BODY_PARTS = {
    0:  "Nose",
    1:  "Neck",
    2:  "RShoulder",
    3:  "RElbow",
    4:  "RWrist",
    5:  "LShoulder",
    6:  "LElbow",
    7:  "LWrist",
    8:  "MidHip",
    9:  "RHip",
    10: "RKnee",
    11: "RAnkle",
    12: "LHip",
    13: "LKnee",
    14: "LAnkle",
    15: "REye",
    16: "LEye",
    17: "REar",
    18: "LEar",
    19: "LBigToe",
    20: "LSmallToe",
    21: "LHeel",
    22: "RBigToe",
    23: "RSmallToe",
    24: "RHeel",
    25: "Background"
}

POSE_BODY_25_BODY_PARTS_CONV = {name: index for index, name in POSE_BODY_25_BODY_PARTS.items()}





class Run:
    def __init__(self):
        self.data = {}
        self.command = ''
        self.name = ''

        self.bridge = CvBridge()  # convert img to cv2
        #self.boxes = None
        self.image = None # image from camera
        self.kpts = None # 25 keypoints from openpose

        self.hand_raised = False

        rospy.init_node('person_recogition')

        self.voice_pub = rospy.Publisher('/voice', String, queue_size=10)
        
        rospy.Subscriber("/camera/rgb/image_raw",
                         Image, self.callback_camera)
        # rospy.Subscriber("/darknet_ros/bounding_boxes",
        #                  BoundingBoxes, self.callback_boxes)
        rospy.Subscriber('pr/op_25kps', String, self.callback_25kpts)

        self.voice('I am ready.')

        rospy.loginfo("ready")

    def callback_25kpts(self, data):
        self.kpts = self._parse_25kps(data.data)

        #self._show_25kps(self.kpts)

    def _parse_25kps(self, data):
        kpts = []
        for i, kpt in enumerate(data.split('\n')):
            if i==0 or kpt == '': 
                continue
            x, y, z = kpt.split()
            kpts.append((float(x), float(y), float(z)))
        return kpts

    def _show_25kps(self, kps):
        pb = POSE_BODY_25_BODY_PARTS_CONV
        shows = {pb['Neck'], pb['RWrist'], pb['LWrist'], pb['MidHip']}
        for i, kpt in enumerate(kps):
            if i not in shows: continue
            print(POSE_BODY_25_BODY_PARTS[i] + ': ' +str(kpt))
        print('\n')

    def filter_kps(self, filter):
        pb = POSE_BODY_25_BODY_PARTS_CONV
        kps = []
        for i, kpt in enumerate(self.kpts):
            if i not in filter: continue
            kps.append(kpt)
        return kps

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


    # def train(self):
    #     people_list = self.data.values() # list of {'faces', []}
    #     people = []
    #     for d in people_list:
    #         people.append(d.get('faces'))
    #     if len(people) > 0:
    #         # create our LBPH face recognizer
    #         self.face_recognizer = cv2.face.LBPHFaceRecognizer_create()
    #         #face_recognizer = cv2.face.EigenFaceRecognizer_create()
    #         #face_recognizer = cv2.face.FisherFaceRecognizer_create()

    #         # prepare data for training
    #         faces = []
    #         labels = []
    #         for k in range(len(people)):
    #             for f in people[k]:
    #                 faces.append(f)
    #                 labels.append(k)

    #         # train our face recognizer of our training faces
    #         self.face_recognizer.train(faces, np.array(labels))
    #         return True
    #     return 


    def callback_camera(self, cam_data):
        try: 
            img = self.bridge.imgmsg_to_cv2(cam_data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.image = img

        self.react_to_command()

    def get_bodybox(self):
        if self.kpts:
            pb = POSE_BODY_25_BODY_PARTS_CONV
            idx = {pb['Neck'], pb['MidHip'], pb['RShoulder'], pb['LShoulder'], pb['RHip'], pb['LHip']}
            points = self.filter_kps(idx)
            xpoints = [int(p[0]) for p in points]
            ypoints = [int(p[1]) for p in points]
            return (min(xpoints), max(xpoints),min(ypoints), max(ypoints))

    def check_hand_raised(self):
        if self.kpts:
            pb = POSE_BODY_25_BODY_PARTS_CONV
            if self.kpts[pb['Neck']][1] != 0:
                if self.kpts[pb['RWrist']][1] != 0 and self.kpts[pb['RWrist']][1] < self.kpts[pb['Neck']][1]:
                    print('put up RIGHT hand')
                    return True
                if self.kpts[pb['LWrist']][1] != 0 and self.kpts[pb['LWrist']][1] < self.kpts[pb['Neck']][1]:
                    print('put up LEFT hand')
                    return True
        return False

    def get_histGBR(self, img):
        pixal = img.shape[0] * img.shape[1]
        
        histSingle0 = cv2.calcHist([img], [0], None, [256], [0, 256])
        histSingle1 = cv2.calcHist([img], [1], None, [256], [0, 256])
        histSingle2 = cv2.calcHist([img], [2], None, [256], [0, 256])

        total = np.concatenate((histSingle0, histSingle1, histSingle2),0)
            
        return (total, pixal)

    # def hist_similar(self, lhist, rhist, lpixal, rpixal):
    #     rscale = rpixal/lpixal
    #     rhist = rhist/rscale
    #     assert len(lhist) == len(rhist)
    #     likely = sum(1 - (0 if l == r else float(abs(l-r))/max(l,r)) for l,r in zip(lhist, rhist)) / len(lhist)
    #     if likely ==1.0:
    #         return [1.0]
    #     return likely

    def hist_similar(self, lhist, rhist):
        return cv2.compareHist(lhist, rhist,0)

    def react_to_command(self):
        
        if self.command == 'meet':
            if self.check_hand_raised():
                
                body = self.get_bodybox()

                if body is not None:
                    if self.data.get(self.name) is None:
                        print("new person")
                        self.data[self.name] = {}
                    else:
                        print("meet again")

                    img = self.image
                    xmin,xmax,ymin,ymax = body
                    self.data[self.name]['body'] = img[ymin:ymax, xmin:xmax]
                    self.command = 'done meet'

                    # Show (for debug)


                    cv2.imshow("body", self.data[self.name]['body'])
                    cv2.waitKey(15)   
                    
            else:
                print("Please raise your hand")

        elif self.command == 'find':
            #pass
            # TH = 80
            img1 = self.image

            unknown_body = self.get_bodybox()

            if unknown_body is not None:
                xmin1,xmax1,ymin1,ymax1 = unknown_body
                cv2.imshow("unknowbody", img1[ymin1:ymax1, xmin1:xmax1])
                cv2.waitKey(5)
                unknowbody_pic = img1[ymin1:ymax1, xmin1:xmax1]
                
            
                names = self.data.keys()

                if len(names) > 0:
                    sim_list=[]
                    for p in names:
                        know_body = self.data[p]['body']

                        testHist, testPixal = self.get_histGBR(know_body)
                        h,w,ch = self.data[p]['body'].shape

                        unknowbody_pic = cv2.resize(unknowbody_pic, (w,h))

                        print know_body.shape, type(know_body), unknowbody_pic.shape, type(unknowbody_pic)

                        targetHist, targetPixal = self.get_histGBR(unknowbody_pic)

                        sim = self.hist_similar(targetHist, testHist)
                        if str(sim) != "[nan]":
                            sim_list.append(sim)
                            rospy.loginfo("similarity of "+p+str(sim))
                        else:
                            sim_list.append(0)
                            rospy.loginfo("similarity of "+p+"[0]")
                    
                    max_sim = max(sim_list)
                    if max_sim>=0.8 and names[sim_list.index(max_sim)]==self.name:
                        self.taskdone('This is ' + self.name +"'s body!")

            # names = self.data.keys()
            # faces, _ = detect_faces(img)

            # if len(faces) > 0:
            #     for f in faces:
            #         # predict the image using our face recognizer
            #         label, confidence = self.face_recognizer.predict(f)
            #         print('name', names[label], 'confidence', confidence)

            #         # Show face (for debug)
            #         cv2.namedWindow("face")
            #         cv2.imshow("face", f)
            #         cv2.waitKey(100)

            #         if confidence < TH:
            #             if names[label] == self.name:
            #                 self.taskdone('This is ' + self.name +
            #                             '. I found you!')

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
