#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from time import sleep

flag = 0
key = ''

class GPS2XYZ:
    def __init__(self):
        rospy.init_node('talker', anonymous=True)
        with open("/home/pranavkdas/janak_WS/src/relocalisation/data/gps_list.txt",'r') as f:
            self.lines = f.readlines()
        self.req = []
        for i in range(len(self.lines)):
            self.req.append(self.lines[i].split("\n"))
        for i in self.req:
            if i==['','']:
                self.req.remove(i)
        for i in range(len(self.req)):
            self.req[i].remove('')
        self.new = []
        for i in range(len(self.req)):
            # print(req[i][0])
            self.new.append(self.req[i][0].split(" "))
        self.dictionary = dict()
        for i in range(len(self.new)):
            self.dictionary[round(float(self.new[i][0]),3)] = self.new[i][:]
        
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        self.sub = rospy.Subscriber('/kitti/oxts/gps/fix',NavSatFix,self.callback)


    
    def callback(self, data):
        # print("Hehe")
        global key, flag
        if not flag:
            # sleep(10)
            key = str(data.latitude)
            flag = 1
            
        self.answer= self.dictionary[round(float(key),3)]
        self.result = self.answer[3]+" "+ self.answer[4] + " "+self.answer[5]
        self.pub.publish(self.result)


if __name__ == '__main__':
    try:
        GPS2XYZ()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
