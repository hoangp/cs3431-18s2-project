import rospy
from std_msgs.msg import String
import pyttsx
                                    
class Speaker:
    '''
    This is the class for agent(robot), who will listen to "voice" topic
    '''
    def __init__(self, pubtopic_name = 'pr/voice', pubtopic_data_class = String):
        rospy.init_node('speaker')
        rospy.Subscriber(pubtopic_name, pubtopic_data_class, self.lis_func)
        rospy.loginfo('Ready to speak voice')
        rospy.spin()
        
    def lis_func(self, data):
        speech = pyttsx.init()
        speech.say(data.data)
        speech.runAndWait()
        rospy.loginfo(data.data)
        
if __name__ == '__main__':
    Speaker()