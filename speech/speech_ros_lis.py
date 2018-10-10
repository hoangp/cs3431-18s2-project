import speech_recognition as sr 
import rospy
from std_msgs.msg import String
import re

class Commander: 

    def __init__(self, cammands_dic = None):
        rospy.init_node('commands_publisher')
        self._pub = rospy.Publisher('commands', String, queue_size=10)
        self._rate = rospy.Rate(1) # 10hz
        
        self.cammands_keys = cammands_dic.keys()
        
    def lis_mic(self, STOP = 'stop'):
       	
        r = sr.Recognizer()  # obtain audio from the microphone
        with sr.Microphone() as source:  
            rospy.loginfo("Please wait. Calibrating microphone...")  
            r.adjust_for_ambient_noise(source, duration=1)  
            rospy.loginfo("Say something!")  
            
            words = ''
            while words.find(STOP) == -1 and not rospy.is_shutdown():
                
                audio = r.listen(source, phrase_time_limit=5)  
                
                try:   # recognize speech using Sphinx 
                    words = r.recognize_google(audio)
                    rospy.loginfo('({})'.format(words))
                    for c in self.cammands_keys:
                        if words.find(c) != -1: 
                            self._pub.publish(words) 
                except sr.UnknownValueError:  
                    #rospy.loginfo("Sphinx could not understand audio")  
                    pass
                except sr.RequestError as e:  
                    rospy.loginfo("Sphinx error; {0}".format(e))  
                    
                    
class CommandListener:
    
    def __init__(self, cammands_dic):
        self.cammands_keys = cammands_dic.keys()
        rospy.init_node('commands_listener')
        rospy.Subscriber('commands', String, self.lis_com)
        rospy.spin()
        rospy.loginfo('listening')
        
        
    def lis_com(self, data):
        rospy.loginfo(data)
        words = data.data
        for c in self.cammands_keys:
            if words.find(c) != -1:
                name = self._find_name(words, c)
                cammands_dic[c](name)
        
    def _find_name(self, msg, c):
        matching = re.findall(r'\b{}\s(\w+)'.format(c), msg)
        print matching
        if matching != []:
            return matching[0]
        
        
        
def meet_person_func(name):
    print('excute <meet_person_func>', name)
    # ...
 
def find_person_func(name):
    print('excure <find_person_func>', name)
    # ...
            
if __name__ == '__main__':
    cammands_dic = {
        'I\'m': meet_person_func,
        'I am': meet_person_func,
        'my name is': meet_person_func,
        'find': find_person_func
    }
    lis = CommandListener(cammands_dic)
