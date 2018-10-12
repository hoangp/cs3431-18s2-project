import speech_recognition as sr 
from cmd_pub import Cmd
import rospy
from std_msgs.msg import String


class SpeechCmd(Cmd): 

    def __init__(self, pubtopic_name = 'cmd', pubtopic_data_class = String):
        Cmd.__init__(self, pubtopic_name, pubtopic_data_class)
       

    def listen(self):
        
        r = sr.Recognizer()  # obtain audio from the microphone
        with sr.Microphone() as source:  
            rospy.loginfo("Please wait. Calibrating microphone...")  
            r.adjust_for_ambient_noise(source, duration=1)  
            rospy.loginfo("Say something!")  
        
            # keep listening
            while not rospy.is_shutdown():
                audio = r.listen(source, phrase_time_limit=5)  
                   
                # speech -> text   
                try:    
                    words = r.recognize_google(audio)
                    rospy.loginfo('({})'.format(words))
                    if self._is_stoping(words):
                        break # stop listening
                    if self._is_allowed_cmds(words): 
                        self._pub.publish(words) 
                        rospy.loginfo('[finished] publish: "{}" on the topic of "{}")'.format(words, self.pubtopic_name))
                except sr.UnknownValueError:  
                    #rospy.loginfo("Sphinx could not understand audio")
                    pass  
                except sr.RequestError as e:  
                    rospy.loginfo("Sphinx error; {0}".format(e))  

   
if __name__ == '__main__':
    cmd_listener = SpeechCmd()
    cmd_listener.listen()
