#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Int16MultiArray, Float32MultiArray
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import math
import random
from scipy.spatial import distance
import array as arr 
import struct
import numpy as np
from time import sleep
from threading import Timer

from wifi_comm_emulation.msg import Conn_Info

class Subscriber:
    def __init__(self, topic, type=Odometry):
        self.sub = rospy.Subscriber(topic, type, self.callback)
        #rospy.loginfo("Subscriber created")
        self.msg = None
        self.msg_received = False
        self.timer = None
        self.timer_initialized = False
        self.id = None
        self.topic = topic

    def timeout(self):
        rospy.logwarn("No message from WAM-V [%s] received for 5 seconds.", self.id )
        self.msg_received = False
    
    def timer_init(self):
        self.timer = Timer(5,self.timeout) # If 5 seconds elapse, call timeout()
        self.timer.start()
        self.timer_initialized = True
        
    def callback(self, msg):
        #rospy.loginfo(rospy.get_caller_id() + " \n I subscribed to [%s] and heard %s", topic, msg.pose.pose)
        if(self.timer_initialized):
            self.timer.cancel()
            self.timer = Timer(5,self.timeout)
            self.timer.start()

            if(not self.msg_received):
                rospy.loginfo('WAM-V [%s] robot found.', self.id)
                self.msg_received = True

        self.msg = msg

class AutoPub:
    def __init__(self, topic, msg_type):
        self.topic_to_pub = topic
        self.pub = rospy.Publisher(self.topic_to_pub, msg_type, queue_size=10)
        self.mgs = msg_type()

class WirelessCommunication:
    def map(self, x, in_min, in_max, out_min=0, out_max=255): 
        in_min = in_min if in_min else 0
        in_max = in_max if in_max else self.wifi_loss_max
        return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

    def rgbGradient(self,minimum, maximum, value):
        minimum, maximum = float(minimum), float(maximum)
        ratio = 2 * (value-minimum) / (maximum - minimum)
        b = int(max(0, 255*(1 - ratio)))
        r = int(max(0, 255*(ratio - 1)))
        g = 255 - b - r

        return r, g, b

    def WifiHeatMap(self, robot_coords, n_robot):
        PI = 3.1415926535
        points = []
        radius = self.heatmap_radius
        step = 5

        for y in np.arange(-radius, radius, step):
            for x in np.arange(-radius, radius, step):
                if(x*x+y*y <= radius*radius):
                    x0 = x + robot_coords[0]
                    y0 = y + robot_coords[1]
                    z0 = 1 + robot_coords[2]
                    loss_coords = (x0,y0,z0)
                    pt = [x0, y0, z0, 0]
                    curr_loss = self.WifiLoss(robot_coords, loss_coords)

                    if(curr_loss < self.wifi_loss_max):
                        r,g,b = self.rgbGradient(0,255,self.map(curr_loss,0,self.wifi_loss_max * 1.26,0,255))
                    if(curr_loss >= self.wifi_loss_max):
                        r,g,b = 240, 40, 0

                    a = 255 
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    pt[3] = rgb

                    points.append(pt)


        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb', 12, PointField.UINT32, 1),
                ]

        header = Header()
        header.frame_id = self.frame_id
        pc2 = point_cloud2.create_cloud(header, fields, points)

        return pc2

    def __init__(self):
        rospy.loginfo("wireless_communication node initialized")

        self.number_of_robos = None
        self.robot_prefix = rospy.get_param('~robot_prefix', 'wamv')
        self.frame_id = rospy.get_param('~frame_id', 'world')

        self.Lc = float(rospy.get_param('~Lc','47.4'))
        self.gain = float(rospy.get_param('~gain','3'))
        self.random_min = float(rospy.get_param('~random_min','2.5'))
        self.random_max = float(rospy.get_param('~random_max','3.5'))

        # lists to keep auto generated subs and pubs
        self.gt_subs = []
        self.pose_subs = []
        self.pose_pubs = []
        self.dist_pubs = []
        self.heatmap_pubs = []
        self.heatmap = rospy.get_param('heatmap',False)

        self.pose_to_sub = ''
        self.gt_to_sub = ''
        self.wifi_sensitivity = float(rospy.get_param('~wifi_sensitivity','-99'))
        self.transmission_power = float(rospy.get_param('~transmission_power','2'))
        self.wifi_loss_max = -(self.wifi_sensitivity - self.transmission_power)
        #print self.wifi_loss_max
        self.heatmap_radius = self.MaxWifiDist(self.wifi_loss_max) #the maximum distantion wifi can send data to

        # get topics names from parameters defined in launch file 
        self.setGtSub(rospy.get_param('~gt_subs_namespace','p3d_wamv'))
        self.setPoseSub(rospy.get_param('~repub_topic_namespace', 'p3d_wamv')) #'robot_localization/odometry/filtered'))
        
        # publisher with number of robots, wifi losses and connections 
        self.conn_info = rospy.Publisher('/' + self.robot_prefix + '/conn_info', Conn_Info,queue_size=10)
        self.conn_msg = Conn_Info()
        self.initTopics()

        #print self.wifi_sensitivity
        #print self.transmission_power        
        
    def SetConnData(self, num_topics):
        self.conn_msg.dist = Float32MultiArray()
        self.conn_msg.loss = Float32MultiArray()
        self.conn_msg.out_power = Float32MultiArray()
        self.conn_msg.published = Int16MultiArray()
        self.conn_msg.dist.data = [0] * num_topics
        self.conn_msg.loss.data = [0] * num_topics
        self.conn_msg.out_power.data = [0] * num_topics
        self.conn_msg.published.data = [0] * num_topics
        
    def setGtSub(self, topic_namespace):
        self.gt_to_sub = topic_namespace

    def getGtSub(self):
        return self.gt_to_sub

    def setPoseSub(self, topic_namespace):
        self.pose_to_sub = topic_namespace

    def getPoseSub(self):
        return self.pose_to_sub

    def getNumRobots(self):
        return self.number_of_robos

    def updateNumRobots(self):
         # Check for all available topics
        pub_dict = dict(rospy.get_published_topics())
        gt_topics = sorted([key for key, value in pub_dict.items() if self.getGtSub() in key.lower()])
        number_of_robos = len(gt_topics) #- 1 #NOTE: maybe search for wamv(number) topic

        return number_of_robos

    def initTopics(self):
        # Check for all available topics
        pub_dict = dict(rospy.get_published_topics())

        # Check for available  ground truth topics
        gt_topics = sorted([key for key, value in pub_dict.items() if self.getGtSub() in key.lower()])

        self.number_of_robos = len(gt_topics) #- 1 #NOTE: maybe search for wamv(number) topic

        # Create subs to ground truth topics
        for topic in gt_topics:
            self.gt_subs.append(Subscriber(topic))
        for i in range(len(gt_topics)): 
            self.gt_subs[i].timer_init()
            self.gt_subs[i].id = i
        
        # Check for available pose topics
        pose_topics = sorted([key for key, value in pub_dict.items() if self.getPoseSub() in key.lower()])
        #print pose_topics

        # Create subs to pose topics
        self.pose_subs = []
        for topic in pose_topics:
            self.pose_subs.append(Subscriber(topic))
        for i in range(len(pose_topics)): 
            self.pose_subs[i].id = i
        
        self.pose_pubs = []
        self.dist_pubs = []
        self.heatmap_pubs = []
        robot_pose_indexes = []

        for i in range(self.number_of_robos):
            robot_pose_indexes.append(gt_topics[i].split("wamv",1)[1][0])

        for i in range(self.number_of_robos):
            for j in range(self.number_of_robos):
                # create pubs for estimated poses
                if i!=j :
                    self.pose_pubs.append(AutoPub(('/' + self.robot_prefix + str(robot_pose_indexes[i]) + '/' + self.robot_prefix + str(robot_pose_indexes[j]) + '/shared_odom'), Odometry))
                    self.dist_pubs.append(AutoPub(('/' + self.robot_prefix + str(robot_pose_indexes[i]) + '/' + self.robot_prefix + str(robot_pose_indexes[j]) + '/pose_gt_dist'), Float32 ))
            if(self.heatmap == True):
                self.heatmap_pubs.append(AutoPub(('/' + self.robot_prefix + str(robot_pose_indexes[i]) + '/' + self.robot_prefix + str(robot_pose_indexes[j]) + '/loss_distribution'), PointCloud2 )) 
        
        rospy.loginfo("Number of robots found: [%s]",self.number_of_robos)
        robot_num = pow(self.getNumRobots(),2) - self.getNumRobots()
        self.SetConnData(robot_num)

    def WifiComm(self):

        if(self.getNumRobots() != self.updateNumRobots()):
            self.initTopics()

        #print self.getNumRobots()

        num = 0  

        for i in range(self.getNumRobots()):
            for j in range(self.getNumRobots()):
                if i != j:
                    r1_x = self.gt_subs[i].msg.pose.pose.position.x
                    r1_y = self.gt_subs[i].msg.pose.pose.position.y
                    r1_z = self.gt_subs[i].msg.pose.pose.position.z

                    r2_x = self.gt_subs[j].msg.pose.pose.position.x
                    r2_y = self.gt_subs[j].msg.pose.pose.position.y
                    r2_z = self.gt_subs[j].msg.pose.pose.position.z

                    r1_coords = (r1_x, r1_y, r1_z)
                    r2_coords = (r2_x, r2_y, r2_z)

                    dist = distance.euclidean(r1_coords, r2_coords)
                    loss = self.WifiLoss(r1_coords, r2_coords)

                    self.dist_pubs[num].pub.publish(dist)

                    # republish msg
                    if loss <= self.wifi_loss_max:
                        msg = self.gt_subs[i].msg
                        self.pose_pubs[num].pub.publish(msg) 
                        # array with connected or not info
                        self.conn_msg.published.data[num] = 1
                    else:
                        self.conn_msg.published.data[num] = 0

                    #publish connections info
                    self.conn_msg.header.frame_id = self.frame_id
                    self.conn_msg.header.stamp = rospy.Time.now()
                    self.conn_msg.dist.data[num] = dist
                    self.conn_msg.loss.data[num] = loss
                    self.conn_msg.out_power.data[num] = self.OutputPower(loss)
                    self.conn_msg.number_of_robos.data = self.getNumRobots()
                    self.conn_info.publish(self.conn_msg)

                    #publish heatmap
                    if(self.heatmap == True):
                        loss_points = self.WifiHeatMap(r1_coords,i)
                        loss_points.header.stamp = rospy.Time.now()
                        self.heatmap_pubs[i].pub.publish(loss_points)

                    num += 1

                    if num > len(self.dist_pubs): 
                        num = 0

            self.conn_info.publish(self.conn_msg)
            if(self.heatmap == True):
                self.heatmap_pubs[i].pub.publish(loss_points)


    def WifiLoss(self, r1_coords, r2_coords, Lc = 47.4):
        dist = distance.euclidean(r1_coords, r2_coords)
        r = random.uniform(self.random_min,self.random_max)
        L = Lc + 10 * r * math.log(dist, 10)
        return L

    # from https://www.link-labs.com/blog/rf-sensitivity-for-m2m
    def OutputPower(self, loss):
        return (self.transmission_power + self.gain - loss)

    def MaxWifiDist(self, wifi_loss):
        dist = math.pow(10, (wifi_loss - self.Lc)/30)
        return dist


if __name__ == '__main__':
    rospy.init_node('wireless_communication', anonymous=True)
    rate = rospy.Rate(5) 

    wireless_communication = WirelessCommunication()

    sleep(2.)

    while not rospy.is_shutdown():
        wireless_communication.WifiComm()
        rate.sleep()
