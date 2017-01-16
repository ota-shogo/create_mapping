#!/usr/bin/env python
import rospy
import time
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData

class hoge(object):
    def __init__(self):
        rospy.init_node('mmap')
        self.sub = rospy.Subscriber('/map', OccupancyGrid, self.callback,queue_size=10)
        self.sub2 = rospy.Subscriber('/map_metadata', MapMetaData, self.callback2,queue_size=10)
        time.sleep(1)
        
    def callback(self, message):
        self.m = list(message.data)

    def callback2(self, message):
        self.w = message.width
        self.h = message.height

    def run(self):
        f = open('oppai.pgm', "w")
        f.write("P2\n")
        f.write("#masaya\n")
        f.write(str(self.w)+" "+str(self.h)+"\n")
        f.write("255\n")
        for i in range(1,int(self.h)):
            for j in range(0,int(self.w)):
                xy = int(self.w) * i - j
                if self.m[xy] == 0:  self.m[xy] = "255"
                if self.m[xy] == -1: self.m[xy] = "0"
                f.write(str(self.m[xy])+"\n")
        f.close()

if __name__ == '__main__':
    t = hoge()
    t.run()

