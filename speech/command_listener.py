import rospy
from std_msgs.msg import String
import re
                  
                    
class CommandListener:
    '''
    This is the class for agent(robot), who will listen to "commands" topic
    '''
    def __init__(self):
        self._focus = False
        self.cammands_dic = {
            'FOCUS ON': self._set_focus_flag,
            'I\'m': self.meet_person_func,
            'I am': self.meet_person_func,
            'my name is': self.meet_person_func,
            'find': self.find_person_func
        }
        self.cammands_keys = self.cammands_dic.keys()
        rospy.init_node('commands_listener')
        rospy.Subscriber('commands', String, self.lis_func)
        rospy.spin()
        
        
    def lis_func(self, data):
        rospy.loginfo(data)
        words = data.data
        if words == 'FOCUS ON':
            _set_focus_flag(True)
        else:
            for c in self.cammands_keys:
                if words.find(c) != -1:
                    name = self._find_name(words, c)
                    cammands_dic[c](name)
        
    def _find_name(self, msg, c):
        matching = re.findall(r'\b{}\s(\w+)'.format(c), msg)
        print matching
        if matching != []:
            return matching[0]
        
    def _set_focus_flag(self, flag):
        _focus = flag
            
    def meet_person_func(self, name):
        # only when self._focus is True, the agent then can take pictures
        if self._focus is True:
            print('excute <meet_person_func>', name)
            # ...
        else:
            # move or turn around to find the person
     
    def find_person_func(self, name):
        print('excure <find_person_func>', name)
        # ...
            
if __name__ == '__main__':
    
    lis = CommandListener()
