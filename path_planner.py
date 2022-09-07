#!/usr/bin/env python3
#Following is the import line for importing the obstacle detection methods i.e. isValidPoint()
#you need to name this node as "path_planner"
#by Aditya Bhat
#referenced https://github.com/zhm-real/PathPlanning for rrt algorithm implementation

from obstacle_detection import isValidPoint
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import math
import numpy as np


class Point2D():
    def __init__(self, pt) -> None:
        self.x=pt[0]
        self.y=pt[1]
        self.parent=None

class RRT():
    def __init__(self, start, end, delta, sample_rate, max_iter) -> None:
        self.start=Point2D(start)
        self.end=Point2D(end)
        self.delta=delta
        self.sample_rate=sample_rate
        self.max_iter=max_iter
        self.vertex=[self.start]
        self.x_range=(-8, 8)
        self.y_range=(4, 4)

    def planning(self):
        for i in range(self.max_iter):
            node_rand = self.generate_random_node(self.sample_rate)
            node_near = self.nearest_neighbor((self.vertex), node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and isValidPoint(node_near.x, node_near.y, node_new.x, node_near.y):
                self.vertex.append(node_new)
                dist=math.hypot(self.end.x - node_new.x, self.end.y - node_new.y)

                if dist <= self.delta and isValidPoint(node_new.x, node_new.y, self.end.x, self.end.y):
                    self.new_state(node_new, self.end)
                    return self.extract_path(node_new)
        return None

    def generate_random_node(self, goal_sample_rate):
        SMALL_NUMBER=1e-6
        if np.random.random() > goal_sample_rate:
            return Point2D((np.random.uniform(self.x_range[0] +SMALL_NUMBER, self.x_range[1] - SMALL_NUMBER),
                         np.random.uniform(self.y_range[0] + SMALL_NUMBER, self.y_range[1] - SMALL_NUMBER)))
        return self.end

    def nearest_neighbor(self, node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist=math.hypot(node_end.x - node_start.x, node_end.y - node_start.y)
        theta=math.atan2(node_end.y - node_start.y, node_end.x - node_start.x)

        dist = min(self.delta, dist)
        node_new = Point2D((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = [(self.end.x, self.end.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path


class Path_Planner(): 
    def __init__(self):
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        # Publishers
        self.pub = rospy.Publisher("planned_path", Path, queue_size=10)

        # Subscribers
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        
        
        self.x_start = (-5.06, -3.12)  # Starting node
        self.x_1 = (-4, -2)  # Goal nodes
        self.x_2 = (-3, 2)
        self.x_3 = (-1, -2)
        self.x_4 = (0.3, 0.2)
        self.x_final = (1.36, -1.8)
    
    
        rrt1 = RRT(self.x_start, self.x_1, 0.5, 0.05, 100000)
        self.path = rrt1.planning()

        rrt2 = RRT(self.x_1, self.x_2, 0.5, 0.05, 10000)
        self.path += rrt2.planning()

        rrt3 = RRT(self.x_2, self.x_3, 0.5, 0.05, 10000)
        self.path += rrt3.planning()

        rrt4 = RRT(self.x_3 ,self.x_4, 0.5, 0.05, 10000)
        self.path += rrt4.planning()

        rrt5 = RRT(self.x_4, self.x_final, 0.5, 0.05, 10000)
        self.path += rrt5.planning()
        
        print(self.path)
    def callback(self, odom):
        pass

    def start(self):
        rospy.loginfo("Node started...")
        while not rospy.is_shutdown():
            self.pub.publish(self.path)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("path_planner", anonymous=True)
    my_node = Path_Planner()
    my_node.start()
