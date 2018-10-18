#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist

PI = math.pi

def move():
    # Starts a new node
    rospy.init_node('robot_movement', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    speed_line = 1.0
    speed_rotate = 30.0 #degree/sec
    angle = 180.0
    distance = 3.0
    isForward = 1#True or False
    clockwise = 1
    
    #Converting from angles to radians
    angular_speed = speed_rotate*2*PI/360
    relative_angle = angle*2*PI/360
    
    while not rospy.is_shutdown():
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        # Checking if our movement is CW or CCW
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)


        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle):
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)


        #Forcing our robot to stop
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        #rospy.spin()
        #Checking if the movement is forward or backwards
        if(isForward):
            vel_msg.linear.x = abs(speed_line)
        else:
            vel_msg.linear.x = -abs(speed_line)
        
        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed_line*(t1-t0)
        #After the loop, stops the robot
        vel_msg.linear.x = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass