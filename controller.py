#!/usr/bin/env python3
# Remember to change the coordinates recieved by the planner from (X, Y) to (X + 1.79, Y + 0.66).
#you need to name this node "omnibase_controller"

#By Aditya Bhat

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

TIME_STEP=0.1

class PID():
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.error = 0
        self.error_last= 0
        self.integral_error=0
        self.derivative_error=0
        self.output=0
    
    def compute(self, current_pos):
        self.error=self.setpoint-current_pos
        self.integral_error+=self.error*TIME_STEP
        self.derivative_error+=(self.error-self.error_last)/TIME_STEP
        self.output= self.Kp*self.error+self.Ki*self.integral_error+self.Kd*self.derivative_error
        return self.output


class tb3_controller():
    def __init__(self):
        # Params
        self.cmd_vel=Twist()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        # Publishers
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Subscribers
        self.sub_odom = rospy.Subscriber("odom", Odometry , self.callback_odom)
        self.sub_planned_path = rospy.Subscriber("planned_path", Point, self.callback_planned_path)

        #PIDs
        self.PID_linear=PID(0,0,0,0)
        self.PID_angular=PID(0,0,0,0)

    def callback_odom(self, data):
        rospy.loginfo()

    def callback_planned_path(self, data):
        rospy.loginfo()

    def start(self):
        rospy.loginfo("Node started...")
        while not rospy.is_shutdown():
            self.pub.publish(self.cmd_vel)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("tb3_controller", anonymous=True)
    my_node = tb3_controller()
    my_node.start()
