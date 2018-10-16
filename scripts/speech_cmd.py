import speech_recognition as sr 
import rospy
from std_msgs.msg import String

class Cmd(object):

    def __init__(self, pubtopic_name, pubtopic_data_class):
        self.pubtopic_name = pubtopic_name
        self.pubtopic_data_class = pubtopic_data_class

        rospy.init_node('command_listner')
        self._pub = rospy.Publisher(self.pubtopic_name, pubtopic_data_class, queue_size=10)
        self._rate = rospy.Rate(1) 

    def _is_allowed_cmds(self, cmds, allowed_all = False):
        allowed_cmds = {
            'I\'m',
            'I am',
            'my name is',
            'this is',
            'meet',
            'find',
            'follow',
            'stop',
            'room',
            'move',
            'stop'
        }
        allowed = allowed_all
        for c in allowed_cmds:
            if cmds.find(c) != -1:
                allowed = True
        return allowed
    
    #added room and move, but not test
    def _check_name(self, words):
        meet_cmds = {
            'i\'m': 'meet',
            'i am'; 'meet',
            'my name is': 'meet',
            'this is': 'meet',
            'meet': 'meet',
            'find': 'find'
            'room': 'room'
            'move': 'move'
        }
        if self._is_allowed_cmds(words.lower()):
            for c in meet_cmds:
                if cmds.find(c) != -1:
                    matching=re.findall(r'\b{}\s(\w+)'.format(c), words)
                    #print('matching: ',matching)
                    if matching != []:
                        name = matching[0]
                        cmds = meet_cmds[c] + ' '+ matching[0]
        return cmds
    
    def _stop_follow(self, words):
        cmds = words.lower()
        if self._is_allowed_cmds(cmds):
            if cmds == 'stop':
                return cmds
            else:
                matching=re.findall(r'\b{}\s(\w+)'.format('stop'), words)
                if matching != []:
                    return False

    def _is_stoping(self, cmds, stop_cmds = 'stop listening'):
        if cmds.find(stop_cmds) != -1:
            return True
        else:
            return False

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
                    #words = r.recognize_google(audio)
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
