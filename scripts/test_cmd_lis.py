import rospy
from std_msgs.msg import String
import re
                  
                    
class CommandListener:
    '''
    This is the class for agent(robot), who will listen to "commands" topic
    '''
    def __init__(self, pubtopic_name = 'cmd', pubtopic_data_class = String):
        self._focus = False
        self.cammands_dic = {
            'FOCUS ON': self._set_focus_flag,
            'I\'m': self.meet_person_func,
            'I am': self.meet_person_func,
            'my name is': self.meet_person_func,
            'meet': self.meet_person_func,
            'find': self.find_person_func
        }
        self.cammands_keys = self.cammands_dic.keys()
        rospy.init_node('commands_listener')
        rospy.Subscriber(pubtopic_name, pubtopic_data_class, self.lis_func)
        rospy.loginfo('Ready to hear commands')
        rospy.spin()
        
        
    def lis_func(self, data):
        rospy.loginfo(data)
        words = data.data
        if words == 'FOCUS ON':
            self._set_focus_flag(True)
        else:
            for c in self.cammands_keys:
                if words.find(c) != -1:
                    name = self._find_name(words, c)
                    rospy.loginfo('Hear from '+ name)
                    self.cammands_dic[c](name)
        
    def _find_name(self, msg, c):
        matching = re.findall(r'\b{}\s(\w+)'.format(c), msg)
        if matching != []:
            return matching[0]
        
    def _set_focus_flag(self, flag):
        _focus = flag
            
    def meet_person_func(self, name):
        # only when self._focus is True, the agent then can take pictures
        if self._focus is True:
            rospy.loginfo('excute <meet_person_func>'+ name)
            # ...
        else:
            rospy.loginfo('Looking for '+ name)
            # move or turn around to find the person
    def find_person_func(self, name):
        rospy.loginfo('excute <find_person_func>'+ name)
        # ...

if __name__ == '__main__':
    
    lis = CommandListener()
