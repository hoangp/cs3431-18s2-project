from cmd_pub import Cmd
import rospy
from std_msgs.msg import String


class StdCmd(Cmd):

    def __init__(self, pubtopic_name = 'cmd', pubtopic_data_class = String):
        Cmd.__init__(self, pubtopic_name, pubtopic_data_class)
       
    def listen(self):
        while True:
            command = raw_input('input:')  # Wait for a user to print something.
            if self._is_stoping(command):
                break # stop listening
            if self._is_allowed_cmds(command): 
                self._pub.publish(command)
                rospy.loginfo('[finished] publish: "{}" on the topic of "{}")'.format(command, self.pubtopic_name))

            rospy.Rate(10).sleep()

   
if __name__ == '__main__':
    cmd_listener = StdCmd()
    cmd_listener.listen()