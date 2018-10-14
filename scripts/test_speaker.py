import rospy
from std_msgs.msg import String
import pyttsx
                                    
class TestSpeaker:
    def __init__(self, pubtopic_name = 'pr/voice', pubtopic_data_class = String):
        rospy.init_node('test_voice')
        self.voice_pub = rospy.Publisher(pubtopic_name, pubtopic_data_class, queue_size=10)
        rospy.loginfo('Ready to publish voice')

def send_cmd_loop(app):
    while True:
        command = raw_input("Say something (Enter empty to quit): ")  # Wait for a user to print something.

        if command == '\n':
            break
        elif command == '':
            break
        else:
            app.voice_pub.publish(command)

        rospy.Rate(10).sleep()


if __name__ == '__main__':
    app = TestSpeaker()
    send_cmd_loop(app)