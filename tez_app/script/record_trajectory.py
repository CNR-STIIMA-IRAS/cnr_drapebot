#!/usr/bin/env python
import rospy
import time

from sensor_msgs.msg import JointState


class csv_saver():
    def __init__(self):
        self.ts = time.time()
        self.sub = rospy.Subscriber("/robot1_mqtt_hw/cmd_joint_pos",JointState,self.callback)
        self.sub2 = rospy.Subscriber("/robot1_mqtt_hw/fb_joint_pos",JointState,self.callback2)

    def callback(self, data):

        stringa = (str(data.header.stamp)
                  + "," + str(data.position[0])
                  + "," + str(data.position[1])
                  + "," + str(data.position[2])
                  + "," + str(data.position[3])
                  + "," + str(data.position[4])
                  + "," + str(data.position[5])
                  + "," + str(data.position[6]) + "\n")

        #print(str(data.header.stamp)
                  #+ "," + str(data.position[0])
                  #+ "," + str(data.position[1])
                  #+ "," + str(data.position[2])
                  #+ "," + str(data.position[3])
                  #+ "," + str(data.position[4])
                  #+ "," + str(data.position[5])
                  #+ "," + str(data.position[6]) + "\n")

        file_name = "traj_cmd_"+str(self.ts)+".txt"
        f = open(file_name, "a")
        f.write(stringa)
        f.close()
        
        
    def callback2(self, data):

        stringa = (str(data.header.stamp)
                  + "," + str(data.position[0])
                  + "," + str(data.position[1])
                  + "," + str(data.position[2])
                  + "," + str(data.position[3])
                  + "," + str(data.position[4])
                  + "," + str(data.position[5])
                  + "," + str(data.position[6]) + "\n")

        #print(str(data.header.stamp)
                  #+ "," + str(data.position[0])
                  #+ "," + str(data.position[1])
                  #+ "," + str(data.position[2])
                  #+ "," + str(data.position[3])
                  #+ "," + str(data.position[4])
                  #+ "," + str(data.position[5])
                  #+ "," + str(data.position[6]) + "\n")

        file_name = "traj_fb_"+str(self.ts)+".txt"
        f = open(file_name, "a")
        f.write(stringa)
        f.close()


def listener():
    rospy.init_node('listener', anonymous=True)
    cs = csv_saver()
    rospy.spin()


if __name__ == '__main__':

    listener()
