import rospy
from std_msgs.msg import String

def get_allowed_cmds():
    allowed_cmds = {
        'list',
        'show',
        'meet',
        'who',
        'find'
    }
    return allowed_cmds

def is_allowed_cmds(command):
    cmd = command.split(' ')
    if len(cmd) == 2:
        command = cmd[0]
    return command in get_allowed_cmds()

class StdCmd:
    def __init__(self, pubtopic_name = 'pr/cmd', pubtopic_data_class = String):
        rospy.init_node('std_cmd')
        self.cmd_pub = rospy.Publisher(pubtopic_name, pubtopic_data_class, queue_size=10)
        rospy.loginfo('Ready to publish command')

def send_cmd_loop(app):
    while True:
        command = raw_input("Input command (Enter empty command to quit): ")  # Wait for a user to print something.

        if command == '\n':
            break
        elif command == '':
            break
        else:
            if is_allowed_cmds(command):
                app.cmd_pub.publish(command)
            else:
                rospy.loginfo('Invalid command')
                print("Valid commands: " + str(get_allowed_cmds()))

        rospy.Rate(10).sleep()

if __name__ == '__main__':
    app = StdCmd()
    send_cmd_loop(app)