#! /usr/bin/python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import pyttsx
from geometry_msgs.msg import Twist
import math

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
        self.command = ''
        self.name = ''

        self.bridge = CvBridge()  # convert ros image to cv2

        self.image = None # cv2 image from camera
        self.kpts = None # list of [25 keypoints from openpose] -> number of person

        self.hand_raised = False
        self.voice_once = False
        self.display_cam = True

        rospy.init_node('person_recogition')

        self.voice_pub = rospy.Publisher('pr/voice', String, queue_size=10)
        self.display_pub = rospy.Publisher('pr/display', Image, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

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

    # def who_cmd(self):
    #     if not self.voice_once:
    #         self.voice("Can you please raise your hand?")
    #         self.voice_once = True

    #     who_raised_hand = self.check_hand_raised()
    #     if who_raised_hand is not None:
    #         body = self.get_bodyboxes(required_all_points = True)[who_raised_hand]
    #         pant = self.get_pantboxes(required_all_points = True)[who_raised_hand]
    #         person = self.get_personboxes(required_all_points = False)[who_raised_hand]

    #         if body is not None and pant is not None:
    #             xmin1,xmax1,ymin1,ymax1 = body
    #             xmin2,xmax2,ymin2,ymax2 = pant
    #             img = self.image
    #             unknowbody_pic = img[ymin1:ymax1, xmin1:xmax1]
    #             unknowpant_pic = img[ymin2:ymax2, xmin2:xmax2]

    #             names = self.data.keys()
    #             sim_list=[]
    #             for p in names:
    #                 body_sim = get_sim(unknowbody_pic, self.data[p]['body'])
    #                 pant_sim = get_sim(unknowpant_pic, self.data[p]['pant'])
    #                 sim_list.append(body_sim + pant_sim)

    #             mostlikely_sim = max(sim_list)

    #             if mostlikely_sim < 1.2:
    #                 mostlikely_name = None
    #                 self.taskdone("I have not meet you before")
    #             else:
    #                 mostlikely_name = names[sim_list.index(mostlikely_sim)] 
    #                 self.taskdone("I think your name is " + mostlikely_name)
    #             self.voice_once = False
    #             self.display(img, person, text=mostlikely_name)

    def who_cmd2(self):
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

        #print(mostlikely_name)
        #print(mostlikely_sim)
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
    
    def goCircle(self, secs):
        rospy.loginfo("Go circle for {0:d} secs".format(secs))
        # Time minus Duration is a Time
        secs_from_now = rospy.Time.now() + rospy.Duration(secs)

        while not rospy.is_shutdown():
            if rospy.Time.now() < secs_from_now:
                self.cmd_vel_pub.publish(genTwist(0, 0, 0, 0, 0, -1))
            else:
                rospy.loginfo("Stop turlebot")
                self.cmd_vel_pub.publish(genTwist(0, 0, 0, 0, 0, 0))
                break 

    def move(self): 
        vel_msg = Twist()

        speed_line = 1.0
        speed_rotate = 30.0 #degree/sec
        angle = 180.0
        distance = 3.0
        isForward = 1#True or False
        clockwise = 1  

        #Converting from angles to radians
        angular_speed = speed_rotate*2*PI/360
        relative_angle = angle*2*PI/360

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # Checking if our movement is CW or CCW
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)

        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        #self.cmd_vel_pub.publish(vel_msg)
        while(current_angle < relative_angle):
            self.cmd_vel_pub.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        #Forcing our robot to stop
        vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(vel_msg)
        #rospy.spin()
        #Checking if the movement is forward or backwards
        # if(isForward):
        #     vel_msg.linear.x = abs(speed_line)
        # else:
        #     vel_msg.linear.x = -abs(speed_line)
        
        # #Setting the current time for distance calculus
        # t0 = rospy.Time.now().to_sec()
        # current_distance = 0

        # #Loop to move the turtle in an specified distance
        # while(current_distance < distance):
        #     #Publish the velocity
        #     self.cmd_vel_pub.publish(vel_msg)
        #     #Takes actual time to velocity calculus
        #     t1=rospy.Time.now().to_sec()
        #     #Calculates distancePoseStamped
        #     current_distance= speed_line*(t1-t0)
        # #After the loop, stops the robot
        # vel_msg.linear.x = 0
        # #Force the robot to stop
        # self.cmd_vel_pub.publish(vel_msg)

        self.taskdone('Done Move')

    def react_to_command(self):
        names = self.data.keys()

        if self.command == 'list':
            self.display_cam = True
            self.list_cmd()
            self.display_cam = True

        elif self.command == 'show':
            self.display_cam = True
            self.show_cmd()

        elif self.command == 'meet':
            self.display_cam = True
            self.meet_cmd()

        elif self.command == 'who':
            #self.display_cam = True
            self.who_cmd2() 
            # if names:
            #     self.who_cmd() 
            # else:
            #     self.taskdone("I have not meet anyone yet")
            #     self.display_cam = True

        elif self.command == 'find':
            self.display_cam = True
            if self.name in names:
                self.find_cmd()
            else:
                self.taskdone("I have not meet " + self.name + ' before')
                self.display_cam = True

        elif self.command == 'move':
            self.move()
            #self.goCircle(3)


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
