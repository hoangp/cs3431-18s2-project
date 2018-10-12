import rospy

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
            'meet',
            'find'
        }
        allowed = allowed_all
        for c in allowed_cmds:
            if cmds.find(c) != -1:
                allowed = True
        return allowed

    def _is_stoping(self, cmds, stop_cmds = 'stop listening'):
        if cmds.find(stop_cmds) != -1:
            return True
        else:
            return False