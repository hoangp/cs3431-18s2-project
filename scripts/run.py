#! /usr/bin/python

import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import pyttsx
from geometry_msgs.msg import Twist
import math
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf

PI = math.pi
BOX_MIN = 5

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
pb = POSE_BODY_25_BODY_PARTS_CONV

def get_header():
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = 'map'
    return h

def genTwist(lx, ly, lz, ax, ay, az):
    twist = Twist()
    twist.linear.x = lx
    twist.linear.y = ly
    twist.linear.z = lz
    twist.angular.x = ax
    twist.angular.y = ay
    twist.angular.z = az
    return twist

def draw_rectangle(img, rect, text=None):
    if text == "?":
        color = (0, 255, 255)
    else:
        color = (0, 255, 0)
    (x, y, w, h) = rect
    cv2.rectangle(img, (x, y), (x+w, y+h), color, 2)
    if text:
        draw_text(img, text, x+5, y+20, color)

def draw_text(img, text, x, y, color):
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.5, color, 2)

def draw_status(img, known, unknown):
    text = "known=" + str(known) + " unknown=" + str(unknown) + " total=" + str(known+unknown)
    cv2.putText(img, text, (5, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

def is_valid_point(p):
    #return (p[0]>100 and p[0]<540 and p[1]>50 and p[1] < 430)
    return sum(p) > 0

def scale_box(box, xmin_factor=1.0, xmax_factor=1.0, ymin_factor=1.0, ymax_factor=1.0):
    xmin,xmax,ymin,ymax = box
    xmin = int(xmin*xmin_factor)
    xmax = int(xmax*xmax_factor)
    ymin = int(ymin*ymin_factor)
    ymax = int(ymax*ymax_factor)
    return (xmin,xmax,ymin,ymax)

def get_histGBR(img):
    pixal = img.shape[0] * img.shape[1]
    
    histSingle0 = cv2.calcHist([img], [0], None, [256], [0, 256])
    histSingle1 = cv2.calcHist([img], [1], None, [256], [0, 256])
    histSingle2 = cv2.calcHist([img], [2], None, [256], [0, 256])

    total = np.concatenate((histSingle0, histSingle1, histSingle2),0)
        
    return (total, pixal)

def hist_similar(lhist, rhist):
    return cv2.compareHist(lhist, rhist,0)

def get_sim(unknowbody_pic, know_body):
    testHist, _ = get_histGBR(know_body)
    h,w,_ = know_body.shape
    h2,w2,_ = unknowbody_pic.shape
    if h2<BOX_MIN or w2<=BOX_MIN:
        return 0
    unknowbody_pic = cv2.resize(unknowbody_pic, (w,h))
    #print know_body.shape, type(know_body), unknowbody_pic.shape, type(unknowbody_pic)
    targetHist, _ = get_histGBR(unknowbody_pic)
    sim = hist_similar(targetHist, testHist)
    if str(sim) == "[nan]":
        return 0
    return sim

class Run:
    def __init__(self):
        self.data = {} # people database
        self.room = {} # room database
        self.current_room = 0
        self.command = ''
        self.name = ''

        self.bridge = CvBridge()  # convert ros image to cv2
        self.tf_listener = tf.TransformListener() #  transfomation between frame

        self.image = None # cv2 image from camera
        self.kpts = None # list of [25 keypoints from openpose] -> number of person

        self.hand_raised = False
        self.voice_once = False
        self.display_cam = True
        self.counter = 0
        self.next_pose = None
        self.prev_pose = None
        self.done_find = False
        self.find_status = None

        rospy.init_node('person_recogition')

        self.voice_pub = rospy.Publisher('pr/voice', String, queue_size=10)
        self.display_pub = rospy.Publisher('pr/display', Image, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        #rospy.Subscriber('camera/color/image_raw', Image, self.callback_camera)
        rospy.Subscriber('pr/camera', Image, self.callback_camera)
        rospy.Subscriber('pr/op_25kps', String, self.callback_25kpts)
        rospy.Subscriber('pr/cmd', String, self.callback_cmd)

        self.voice('I am ready.')
        rospy.loginfo("ready")

    def callback_25kpts(self, data):
        self.kpts = self._parse_25kps(data.data)
        #print("callback_25kpts")
        #print(len(self.kpts))
        #for kpts in self.kpts:
        #    self.show_kps(kpts, range(26))

    def _parse_25kps(self, data):
        kpts = []
        #print(len(data.split('\n')))
        for i, kpt in enumerate(data.split('\n')):
            if i==0 or kpt == '': 
                continue
            x, y, z = kpt.split()
            kpts.append((float(x), float(y), float(z)))

        num_person = len(kpts) // 25
        person = []
        if num_person > 0:
            for i in range(num_person):
                person.append(kpts[i*25:(i+1)*25-1])
        return person

    def show_kps(self, kpts, shows):
        for i, kpt in enumerate(kpts):
            if i not in shows: continue
            print(POSE_BODY_25_BODY_PARTS[i] + ': ' +str(kpt))
        print('\n')

    def voice(self, text):
        # import pyttsx
        # speech = pyttsx.init()
        # speech.say(text)
        # speech.runAndWait()
        engine = pyttsx.init()
        rate = engine.getProperty("rate")
        engine.setProperty("rate", rate - 60)
        engine.setProperty("voice", 'english+f3')
        engine.say(text)
        engine.runAndWait()

        self.voice_pub.publish(text)
        rospy.loginfo(text)

    def taskdone(self, text):
        self.display_cam = False
        self.command = ''  # reset to empty
        self.voice(text)
        
    def callback_camera(self, cam_data):
        try: 
            img = self.bridge.imgmsg_to_cv2(cam_data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.image = img

        #print(self.get_pose())

        if self.display_cam:
            #self.display(img, [],draw_box = False )
            self.display_pub.publish(cam_data)

        # Show (for debug)  
        #cv2.imshow("camera",img)
        #cv2.waitKey(15)

        self.react_to_command()

    def callback_cmd(self, command):
        rospy.loginfo("Received command '" + command.data + "'")
        cmd = command.data.split(' ')
        if len(cmd) == 2:
            self.command = cmd[0]
            self.name = cmd[1]
        else:
            self.command = command.data

    def filter_kps(self, kpts, filter):
        kps = []
        for i, kpt in enumerate(kpts):
            if i not in filter: continue
            kps.append(kpt)
        return kps

    def get_box(self, kpts, idx, required_all_points):
        if self.kpts:
            points = self.filter_kps(kpts, idx)
            #print(len(points))
            if required_all_points:
                for p in points:
                    if not is_valid_point(p): 
                        return None
            xpoints = [p[0] for p in points if p[0]>0]
            ypoints = [p[1] for p in points if p[1]>0]

            if xpoints and ypoints:
                box = (min(xpoints), max(xpoints), min(ypoints), max(ypoints))
                return scale_box(box)

            return None

    def get_bodyboxes(self, required_all_points):
        idx = {pb['Neck'], pb['MidHip'], pb['RShoulder'], pb['LShoulder'], pb['RHip'], pb['LHip']}
        #print("bodybox")
        #self.show_kps(idx)
        return [self.get_box(kpts, idx, required_all_points) for kpts in self.kpts]

    def get_pantboxes(self, required_all_points):    
        idx = {pb['MidHip'], pb['RHip'], pb['LHip'], pb['RKnee'], pb['LKnee']}
        #print("pantbox")
        #self.show_kps(idx)
        return [self.get_box(kpts, idx, required_all_points) for kpts in self.kpts]

    def get_personboxes(self, required_all_points):
        idx = range(25)
        #print("bodybox")
        #self.show_kps(idx)
        return [scale_box(self.get_box(kpts, idx, required_all_points),ymin_factor=0.90) for kpts in self.kpts]

    def check_hand_raised(self):
        for i, kpts in enumerate(self.kpts):
            if kpts[pb['Neck']][1] != 0:
                if kpts[pb['RWrist']][1] != 0 and kpts[pb['RWrist']][1] < kpts[pb['Neck']][1]:
                    #print('put up RIGHT hand')
                    return i
                if kpts[pb['LWrist']][1] != 0 and kpts[pb['LWrist']][1] < kpts[pb['Neck']][1]:
                    #print('put up LEFT hand')
                    return i
        return None

    def list_cmd(self):
        if self.data:
            names = self.data.keys()
            text = "In my memory, there are " + str(len(names)) + " people: "
            for name in names:
                text = text + ", " + name
            self.taskdone(text)
        else:
            self.taskdone("there's no ones in my memory")
            
    def show_cmd(self):
        if self.data.get(self.name) is None:
            self.taskdone("there's no " + self.name + " in my memory")
        else:
            #cv2.imshow("shirt of " + self.name, self.data[self.name]['body'])
            #cv2.imshow("pant of " + self.name, self.data[self.name]['pant'])
            #cv2.waitKey(15)
            img = self.data[self.name]['person']
            self.taskdone("Im my memory, this is " + self.name)
            self.display(img, [], draw_box=False )

    def display(self, img, box, draw_box = True, text=None):
        if draw_box:
            xmin,xmax,ymin,ymax = box
            rect = (xmin,ymin,xmax-xmin,ymax-ymin)
            draw_rectangle(img, rect, text=text)
        imgmsg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.display_pub.publish(imgmsg)

    def display_status(self, img, known, unknown):
        draw_status(img, known, unknown)
        imgmsg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.display_pub.publish(imgmsg)

    def meet_cmd(self):
        if not self.voice_once:
            self.voice(self.name + " Can you please raise your hand?")
            self.voice_once = True

        who_raised_hand = self.check_hand_raised()
        if who_raised_hand is not None:
            body = self.get_bodyboxes(required_all_points = True)[who_raised_hand]
            pant = self.get_pantboxes(required_all_points = True)[who_raised_hand]
            person = self.get_personboxes(required_all_points = False)[who_raised_hand]

            if body is not None and pant is not None:
                xmin1,xmax1,ymin1,ymax1 = body
                xmin2,xmax2,ymin2,ymax2 = pant
                w1 = xmax1 - xmin1
                w2 = xmax2 - xmin2
                h1 = ymax1 - ymin1
                h2 = ymax2 - ymin2
                if w1>BOX_MIN and w2>BOX_MIN and h1>BOX_MIN and h2>BOX_MIN:
                    if self.data.get(self.name) is None:
                        self.taskdone("Hello " + self.name + " How are you")
                        self.data[self.name] = {}
                    else:
                        self.taskdone("Hello " + self.name + " we meet again")
                    self.voice_once = False

                    img = self.image
                    self.data[self.name]['body'] = img[ymin1:ymax1, xmin1:xmax1]
                    self.data[self.name]['pant'] = img[ymin2:ymax2, xmin2:xmax2]

                    xmin,xmax,ymin,ymax = person
                    self.data[self.name]['person'] = img[ymin:ymax, xmin:xmax]

                    import copy
                    imgcopy = copy.deepcopy(img)
                    self.display(imgcopy, person, text=self.name)

                # Show (for debug)
                #cv2.imshow("shirt", self.data[self.name]['body'])
                #cv2.imshow("pant", self.data[self.name]['pant'])
                #cv2.waitKey(15)

            # else:
            #     print("please stand in front of the camera")
                
        # else:
        #     print("Please raise your hand")

    def who_cmd(self):
        bodies = self.get_bodyboxes(required_all_points = True)
        pants = self.get_pantboxes(required_all_points = True)
        persons = self.get_personboxes(required_all_points = False)

        known_counter = 0
        unknown_counter = 0

        img = self.image

        for i in range(len(bodies)):
            body = bodies[i]
            pant = pants[i]
            person = persons[i]
            if body is not None and pant is not None:
                xmin1,xmax1,ymin1,ymax1 = body
                xmin2,xmax2,ymin2,ymax2 = pant
                unknowbody_pic = img[ymin1:ymax1, xmin1:xmax1]
                unknowpant_pic = img[ymin2:ymax2, xmin2:xmax2]

                names = self.data.keys()
                mostlikely_sim = 0

                if names:
                    sim_list=[]
                    for p in names:
                        body_sim = get_sim(unknowbody_pic, self.data[p]['body'])
                        pant_sim = get_sim(unknowpant_pic, self.data[p]['pant'])
                        sim_list.append(body_sim + pant_sim)
                    mostlikely_sim = max(sim_list)

                if mostlikely_sim < 1.2:
                    self.display(img, person, text="?")
                    unknown_counter += 1
                else:
                    mostlikely_name = names[sim_list.index(mostlikely_sim)]     
                    self.display(img, person, text=mostlikely_name)
                    known_counter += 1

        self.display_status(img,known_counter,unknown_counter)

    def find_cmd(self):
        img = self.image
        bodies = self.get_bodyboxes(required_all_points=True)
        pants = self.get_pantboxes(required_all_points=True)
        persons = self.get_personboxes(required_all_points=False)

        mostlikely_sim = []
        mostlikely_name = []
        for i in range(len(bodies)):
            if bodies[i] and pants[i]: 
                xmin1,xmax1,ymin1,ymax1 = bodies[i]
                xmin2,xmax2,ymin2,ymax2 = pants[i]
                #xmin3,xmax3,ymin3,ymax3 = persons[i]
                unknowbody_pic = img[ymin1:ymax1, xmin1:xmax1]
                unknowpant_pic = img[ymin2:ymax2, xmin2:xmax2]
                #unknowperson_pic = img[ymin3:ymax3, xmin3:xmax3]

                # Show (for debug)
                #cv2.imshow("unknown body " +str(i), unknowbody_pic)
                #cv2.imshow("unknown pant " +str(i), unknowpant_pic)
                #cv2.imshow("unknown person" +str(i), unknowperson_pic)
                #cv2.waitKey(15)                       

                names = self.data.keys()
                sim_list=[]
                for p in names:
                    body_sim = get_sim(unknowbody_pic, self.data[p]['body'])
                    pant_sim = get_sim(unknowpant_pic, self.data[p]['pant'])
                    sim_list.append(body_sim + pant_sim)

                mostlikely_sim.append ( max(sim_list) )
                mostlikely_name.append ( names[sim_list.index(max(sim_list))] )

                if max(sim_list) < 1.2:
                    self.display(img, persons[i], text="?")
                else:
                    mostlikely_name_str = names[sim_list.index(max(sim_list))]
                    self.display(img, persons[i], text=mostlikely_name_str)
            
        if self.name in mostlikely_name:
            idx = mostlikely_name.index(self.name)
            sim = mostlikely_sim[idx]
            if sim > 1.4:
                self.taskdone("I found " + self.name + " here")
                self.display(img, persons[idx], text=self.name)
                # Show person
                #xmin3,xmax3,ymin3,ymax3 = persons[idx]
                #cv2.imshow("found person", img[ymin3:ymax3, xmin3:xmax3])
                #cv2.waitKey(15)

    def check_goal(self, goal):
        if goal is None:
            return True
        p = self.get_pose().position
        g = goal.position
        
        limit = 0.1
        print('check goal')
        print( abs(p.x-g.x) + abs(p.y - g.y) )
        return abs(p.x-g.x) < limit and abs(p.y - g.y) < limit

    def check_stop(self):
        pose = self.get_pose()

        if self.prev_pose is None:
            self.prev_pose = pose
            return False

        p = pose.position
        v = self.prev_pose.position

        self.prev_pose = pose
        zero = 0.1
        print('check stop')
        print( abs(p.x-v.x) + abs(p.y - v.y) )
        return abs(p.x-v.x) < zero and abs(p.y - v.y) < zero

    def find_cmd2(self):
        if self.find_status is None or self.find_status == "finding":
            if not self.voice_once:
                self.voice("Try to find " + self.name + " in room " + self.room.keys()[self.current_room])
                self.voice_once = True

            self.counter += 1
            self.cmd_vel_pub.publish(genTwist(0, 0, 0, 0, 0, -0.3))

            if self.counter >= 15:
                if self.current_room >= len(self.room.keys()) - 1:
                    self.current_room = 0
                else:
                    self.current_room += 1
                room_name = self.room.keys()[self.current_room]
                self.next_pose = self.room[room_name]

                self.goal_pub.publish(PoseStamped(get_header(), self.next_pose))
                self.voice("Move to room " + room_name)
                self.counter = 0
                self.voice_once = False

                self.find_status = "moving"

        elif self.find_status == "moving":
            if self.check_goal(self.next_pose):
                self.next_pose = None
                self.find_status = "force stop"

        elif self.find_status == "force stop":
            self.goal_pub.publish(PoseStamped(get_header(), self.get_pose()))
            self.cmd_vel_pub.publish(genTwist(0, 0, 0, 0, 0, 0))
            if self.check_stop():
                if self.done_find:
                    self.command = ''
                    self.done_find = False
                else:
                    self.find_status = "finding"

        if self.command == 'find':
            self.find_cmd()

            if self.command == '':
                self.goal_pub.publish(PoseStamped(get_header(), self.get_pose()))
                self.cmd_vel_pub.publish(genTwist(0, 0, 0, 0, 0, 0))
                self.counter = 0
                self.voice_once = False
                self.next_pose = None
                self.done_find = True
                self.command = 'find'
                self.find_status = 'force stop'

        # if self.force_stop:
        #     if not self.check_stop():
        #         self.goal_pub.publish(PoseStamped(get_header(), self.get_pose()))
        #         self.cmd_vel_pub.publish(genTwist(0, 0, 0, 0, 0, 0))
        #     else:
        #         self.command = ''
        #         self.force_stop = False
        # else:
        # if self.counter >= 15:
        #     if self.current_room >= len(self.room.keys()) - 1:
        #         self.current_room = 0
        #     else:
        #         self.current_room += 1

        #     room_name = self.room.keys()[self.current_room]
        #     self.next_pose = self.room[room_name]

        #     self.goal_pub.publish(PoseStamped(get_header(), self.next_pose))
        #     self.voice("Move to room " + room_name)
        #     self.counter = 0
        #     self.voice_once = False

        # elif self.check_goal(self.next_pose):
        #     self.next_pose = None
        #     # stop
        #     if self.check_stop():
        #         self.prev_pose = None    
        #         if not self.voice_once:
        #             self.voice("Try to find " + self.name + " in room " + self.room.keys()[self.current_room])
        #             self.voice_once = True
        #         print(self.counter)
        #         self.counter += 1
        #         self.cmd_vel_pub.publish(genTwist(0, 0, 0, 0, 0, -0.3))
        #     else:
        #         self.goal_pub.publish(PoseStamped(get_header(), self.get_pose()))
        #         self.cmd_vel_pub.publish(genTwist(0, 0, 0, 0, 0, 0))

        # self.find_cmd()

        # if self.command == '':
        #     self.goal_pub.publish(PoseStamped(get_header(), self.get_pose()))
        #     self.cmd_vel_pub.publish(genTwist(0, 0, 0, 0, 0, 0))
        #     self.counter = 0
        #     self.voice_once = False
        #     self.next_pose = None
        #     #self.force_stop = True
        #     #self.command = 'find'
                    
 
    def react_to_command(self):
        names = self.data.keys()

        if self.command == 'list':
            self.list_cmd()
            self.display_cam = True

        elif self.command == 'show':
            self.show_cmd()

        elif self.command == 'meet':
            self.display_cam = True
            self.meet_cmd()

        elif self.command == 'who':
            self.who_cmd() 

        elif self.command == 'find':
            if self.name in names:
                self.find_cmd2()
            else:
                self.taskdone("I have not meet " + self.name + ' before')
                self.display_cam = True

        elif self.command == 'room':
            rooms = self.room.keys()
            if self.room.get(self.name):
                self.current_room = rooms.index(self.name)
            else:
                self.current_room = len(self.room.keys())

            self.room[self.name] = self.get_pose()
            self.taskdone("Remember room " + self.name)
            self.display_cam = True
            print(self.room[self.name])

        elif self.command == 'move':
            rooms = self.room.keys()
            if self.name in rooms:
                self.goal_pub.publish(PoseStamped(get_header(), self.room[self.name]))
                self.current_room = rooms.index(self.name)
                self.taskdone("Move to room " + self.name)
            else:
                self.taskdone("I do not know where room " + self.name + " is")
            self.display_cam = True

        elif self.command == 'stop':
            self.goal_pub.publish(PoseStamped(get_header(), self.get_pose()))
            self.cmd_vel_pub.publish(genTwist(0, 0, 0, 0, 0, 0))
            self.taskdone("stop")
  
        elif self.command == 'pose':
            print(self.get_pose())
            self.taskdone("Printed location of room")
            self.display_cam = True

        elif self.command == 'circle':
            self.cmd_vel_pub.publish(genTwist(0, 0, 0, 0, 0, -0.3))
            self.counter += 1
            print(self.counter)
            if self.counter == 15:
                self.cmd_vel_pub.publish(genTwist(0, 0, 0, 0, 0, 0))
                self.counter = 0
                self.taskdone("done circle")
            self.display_cam = True

    def get_robot_transform(self):
        map_frame = '/map'
        robot_frame = '/base_link'
        # getLatestCommonTime(source_frame, target_frame) -> time
        # Determines that most recent time for which Transformer can compute the transform 
        # between the two given frames, that is, between source_frame and target_frame. 
        # Returns a rospy.Time. 
        self.tf_listener.waitForTransform(map_frame, robot_frame, rospy.Time(), rospy.Duration(5))
        t = self.tf_listener.getLatestCommonTime(robot_frame, map_frame) 
        # lookupTransform(target_frame, source_frame, time) -> position, quaternion
        # Returns the transform from source_frame to target_frame at the time time. 
        # time is a rospy.Time instance. 
        # Returned as position (x, y, z) and an orientation quaternion (x, y, z, w). 
        position, orientation = self.tf_listener.lookupTransform(map_frame, robot_frame, t)
        return position, orientation

    def get_pose(self):
        #rospy.wait_for_service('get_robot_pose')
        # try:
        #     srv = rospy.ServiceProxy('get_robot_pose', GetRobotPose)
        #     response = srv()

        p, o = self.get_robot_transform()
        #rp = RobotPose(p[0], p[1], p[2], o[0], o[1], o[2], o[3])
        #rospy.loginfo('Returning robot position (%f, %f, %f) orientation (%f, %f, %f, %f)' %(
        #rp.px, rp.py, rp.pz, rp.ox, rp.oy, rp.oz, rp.ow))
        #return GetRobotPoseResponse(rp)

        # convert RobotPose to Pose
        #rp = response.pose 
        #position = Point(rp.px, rp.py, rp.pz)
        #orientation = Quaternion(rp.ox, rp.oy, rp.oz, rp.ow)

        position = Point(p[0], p[1], p[2])
        orientation = Quaternion(o[0], o[1], o[2], o[3])
        return Pose(position, orientation)
        # except rospy.ServiceException, e:
        #     print "Service call failed: %s" % e


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
