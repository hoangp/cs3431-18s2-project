import speech_recognition as sr 
import rospy
from std_msgs.msg import String

class MicCommander: 

    def __init__(self):
        self.cammands_keys = {
            'I\'m',
            'I am',
            'my name is',
            'find'
        }
        rospy.init_node('commands_publisher')
        self._pub = rospy.Publisher('commands', String, queue_size=10)
        self._rate = rospy.Rate(1) # 10hz
        
        
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
   
if __name__ == '__main__':
    com = MicCommander()
    com.lis_mic()
