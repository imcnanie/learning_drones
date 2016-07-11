#!/usr/bin/env python  
import roslib
import rospy
import tf
import std_msgs
from math import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Joy, Image
import geometry_msgs.msg
import mavros
import mavros_msgs.srv
from mavros import setpoint as SP
from mavros import command
from threading import Thread
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import time
import threading
import thread

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

from random import randint

print "broadcasting"

import utm

IS_APM = False

class rcOverride:
    def __init__(self, copter_id = "1", mavros_string="/mavros/copter1"):
        rospy.init_node('rc_override'+copter_id)
        mavros.set_namespace(mavros_string)  # initialize mavros module with default namespace

        self.rc = OverrideRCIn()
        self.override_pub = rospy.Publisher(mavros_string+"/rc/override", OverrideRCIn, queue_size=10)

    def get_RC(self):
        return self.rc
        
    def clean_RC(self):
        RC = []
        for x in len(range(self.rc)):
            RC = 1500
        self.override_pub.publish(RC)
        
    def ruin_RC(self):
        RC = []
        for x in self.rc:
            RC.append(randint(1000,2000))
        self.override_pub.publish(RC)
        
    def publish_RC(self, RC):
        self.override_pub.publish(RC)
        

class posVel:
    def __init__(self, copter_id = "1"):
        self.copter_id = copter_id
        mavros_string = "/mavros"
        #rospy.init_node('velocity_goto_'+copter_id)
        mavros.set_namespace(mavros_string)  # initialize mavros module with default namespace



        self.mavros_string = mavros_string

        self.final_alt = 0.0
        self.final_pos_x = 0.0
        self.final_pos_y = 0.0        
        self.final_vel = 0.0
        
        self.cur_rad = 0.0
        self.cur_alt = 0.0
        self.cur_pos_x = 0.0
        self.cur_pos_y = 0.0
        self.cur_vel = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.avz = 0.0

        self.pose_open = []

        self.alt_control = True
        self.override_nav = False
        self.reached = True
        self.done = False

        self.last_sign_dist = 0.0

        # for local button handling
        self.click = " "
        self.button_sub = rospy.Subscriber("abpause_buttons", std_msgs.msg.String, self.handle_buttons)

        # publisher for mavros/copter*/setpoint_position/local
        self.pub_vel = SP.get_pub_velocity_cmd_vel(queue_size=10)
        # subscriber for mavros/copter*/local_position/local
        self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'local'), SP.PoseStamped, self.temp)

        self.cv_bridge = CvBridge()
        self.image_data = open('image_data.txt','w')
        
    def handle_buttons(self, msg):
        self.click = str(msg)[6:]

    def temp(self, topic):
        pass

    def start_subs(self):
        pass

    def update(self, com_x, com_y, com_z):
        self.alt_control = True
        self.reached = False
        self.override_nav = False
        self.final_pos_x = com_x
        self.final_pos_y = com_y
        self.final_alt = com_z

    def set_velocity(self, vel_x, vel_y, vel_z):
        self.override_nav = True
        self.vx = vel_x
        self.vy = vel_y
        self.vz = vel_z

    def subscribe_pose(self):
        rospy.Subscriber(self.mavros_string+'/global_position/local',
                         Odometry,
                         self.handle_pose)
         
        rospy.spin()

    def subscribe_pose_thread(self):
        s = Thread(target=self.subscribe_pose, args=())
        s.daemon = True
        s.start()

    def arm(self):
        arm = rospy.ServiceProxy(self.mavros_string+'/cmd/arming', mavros_msgs.srv.CommandBool)  
        print "Arm: ", arm(True)
        
    def disarm(self):
        arm = rospy.ServiceProxy(self.mavros_string+'/cmd/arming', mavros_msgs.srv.CommandBool)  
        print "Disarm: ", arm(False)

    def setmode(self,base_mode=0,custom_mode="OFFBOARD",delay=0.1):
        set_mode = rospy.ServiceProxy(self.mavros_string+'/set_mode', mavros_msgs.srv.SetMode)  
        if IS_APM:
            if custom_mode == "OFFBOARD":
                custom_mode = "GUIDED"
            if custom_mode == "AUTO.LAND":
                custom_mode = "LAND"
            if custom_mode == "MANUAL":
                custom_mode = "STABILIZE"
            if custom_mode == "POSCTL":
                custom_mode = "LOITER"
        ret = set_mode(base_mode=base_mode, custom_mode=custom_mode)
        print "Changing modes: ", ret
        time.sleep(delay)

    def takeoff_velocity(self, alt=7):
        self.alt_control = False
        while self.cur_alt < alt - 1:
            print "CUR ALT: ", self.cur_alt, "GOAL: ", alt
            #self.set_velocity(0, 0, 1.5)
            self.update(self.cur_pos_x, self.cur_pos_y, alt)
 
        time.sleep(0.1)
        #self.set_velocity(0, 0, 0)

        self.final_alt = alt
        
        rospy.loginfo("Reached target Alt!")

    def handle_pose(self, msg):
        pos = msg.pose.pose.position
        qq = msg.pose.pose.orientation

        self.pose_open = qq

        q = (msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w)

        euler = euler_from_quaternion(q)

        self.cur_rad = euler[2]

        self.cur_pos_x = pos.x 
        self.cur_pos_y = pos.y
        self.cur_alt = pos.z

    def controller_callback(self, data):
        self.vx = -data.axes[0]
	self.vy = data.axes[1]
        self.vz = -data.axes[2]
        self.avz = data.axes[3]
        print "GOT: ", self.vx, self.vy
        
    def controller_sub(self):
        rospy.Subscriber("joy", Joy, self.controller_callback)
        
    def navigate(self):
        rate = rospy.Rate(30)   # 30hz
        magnitude = 1.0  # in meters/sec

        msg = SP.TwistStamped(
            header=SP.Header(
                frame_id="base_footprint",  # doesn't matter
                stamp=rospy.Time.now()),    # stamp should update
        )
        while not rospy.is_shutdown():
            msg.twist.linear = geometry_msgs.msg.Vector3(self.vx*magnitude, self.vy*magnitude, self.vz*magnitude)
            #msg.twist.angular = geometry_msgs.msg.Vector3(0.0, 0.0, self.avz*magnitude)
            self.pub_vel.publish(msg)
            #print self.vx, self.vy
            rate.sleep()

    def image_callback(self, msg):
        #print "Header: ", msg.header.seq
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            pass #print(e)
        else:
            # Save your OpenCV2 image as a jpeg
            im_name = 'kinect_image_'+str(msg.header.seq)+'.jpeg'
            if self.vx != 0.0 and self.vy != 0.0:
                cv2.imwrite('frames/'+im_name, cv2_img)
                self.image_data.write(im_name+' : x:'+str(self.vx)+',y:'+str(self.vy)+',z:'+str(self.vz)+',az:'+str(self.avz)+'\n')
            
    def record(self):
         # Define your image topic
        image_topic = "/kinect/depth/image_raw"
        # Set up your subscriber and define its callback
        rospy.Subscriber(image_topic, Image, self.image_callback)
        # Spin until ctrl + c
        rospy.spin()
            
    def land_velocity(self):
        while self.cur_alt > 7.0:
            self.update(self.home_lat, self.home_lon, 7.0)
            print "landing: ", self.cur_alt
        self.update(self.home_lat, self.home_lon, 5.0)
        alts = 5.0
        while self.cur_alt < 0.45:
            alts = alts - 0.1
            self.update(self.home_lat, self.home_lon, alts)
            print "slowly landing: ", self.cur_alt
            time.sleep(0.5)
        print "Landed, disarming"
        self.update(self.home_lat, self.home_lon, 0)
            
    def get_copter_id(self):
        return self.copter_id
            
    def get_lat_lon_alt(self):
        return (self.cur_pos_x, self.cur_pos_y, self.cur_alt)

    def get_home_lat_lon_alt(self):
        return (self.home_lat, self.home_lon, self.home_alt)

    def start_navigating(self):
        t = Thread(target = self.navigate, args = ())
        t.daemon = True
        t.start()

    def start_recording(self):
        r = Thread(target = self.record, args = ())
        r.daemon = True
        r.start()

class SmartRTL:
    def __init__(self, copters):
        self.initial_alt_drop = 5
        self.copters = copters
        self.sorted_copters = []
        copters_by_alt = {}
        for cop in copters:
            copters_by_alt[cop] = cop.get_lat_lon_alt()[-1]

        self.sorted_copters = sorted(copters_by_alt)
                
        print "SORTED COPTERS", [c.get_copter_id() for c in self.sorted_copters]

        for w in self.sorted_copters[::-1][:-1]:
            self.raise_cops(w)
            
        for x in self.sorted_copters:
            self.land_cop(x)

    def raise_cops(self,cop):
        cur_pos_x, cur_pos_y, cur_alt = cop.get_lat_lon_alt()
        self.raise_height = cur_alt + 5
        cop.update(cur_pos_x, cur_pos_y, self.raise_height)
        while cur_alt < self.raise_height-1:
            cur_pos_x, cur_pos_y, cur_alt = cop.get_lat_lon_alt()
            print "Copter", cop.copter_id, " altitude: ",cur_alt
        #cop.set_velocity(0.0,0.0,0.0)
    
    def land_cop(self,cop):
        cur_pos_x, cur_pos_y, cur_alt = cop.get_lat_lon_alt()
        home_lat, home_lon, home_alt = cop.get_home_lat_lon_alt()
        
        print "RTLing Copter", cop.copter_id
        self.drop_height = cur_alt-self.initial_alt_drop
        
        print "Copter", cop.copter_id, " dropping..."
        time.sleep(1)
        
        cop.update(cur_pos_x, cur_pos_y, self.drop_height)
        while cur_alt > self.drop_height+2.0:
            cur_pos_x, cur_pos_y, cur_alt = cop.get_lat_lon_alt()
            print "cur_alt: ", cur_alt, "check", self.drop_height+1.0
            #print "Copter", cop.copter_id, " altitude: ",cur_alt
        #cop.update(0.0,0.0,0.0)
        
        print "Copter", cop.copter_id, "going to home location..."
        time.sleep(1)
        
        cop.update(home_lat, home_lon, self.drop_height)
        while not cop.reached:
            cur_pos_x, cur_pos_y, cur_alt = cop.get_lat_lon_alt()
            time.sleep(0.025)
            #cop.update(home_lat, home_lon, self.drop_height)
            print "Copter", cop.copter_id, " drop height", self.drop_height," altitude: ", cur_alt
        #cop.update(0.0,0.0,0.0)
        cop.setmode(custom_mode="AUTO.LAND")
        #cop.land_velocity()

class SafeTakeoff:
    def __init__(self, copters, offsets_x, offsets_y, alt = 20.0):
        self.cops = copters

        self.offs_x = offsets_x
        self.offs_y = offsets_y
        
        self.ids = []
        for i in range(len(self.cops)):
            self.ids.append(i)

        self.offs_hype = []
        for o in range(len(self.cops)):
            h = sqrt(self.offs_y[o] **2.0 + self.offs_x[o] **2.0)
            self.offs_hype.append(h)

        self.alt = alt

        #running_x = 0.0
        #running_y = 0.0
        #for cop in copters:
        #    running_x = running_x + cop.cur_pos_x
        #    running_y = running_y + cop.cur_pos_y

        #self.center_x = running_x / float(len(copters))
        #self.center_y = running_y / float(len(copters))

        self.sorted_ids = [x for (y,x) in sorted(zip(self.offs_hype, self.ids))]

        self.center_x = self.cops[0].cur_pos_x
        self.center_y = self.cops[0].cur_pos_y

        for i in self.sorted_ids[::-1]:
            self.takeoff_cop(i)
            
    def takeoff_cop(self, id):
        self.cops[id].setmode(custom_mode = "OFFBOARD")
        self.cops[id].arm()

        time.sleep(0.25)

        self.cops[id].takeoff_velocity(alt = self.alt)
        #self.cops[id].update(self.center_x + self.offs_x[id], self.center_y + self.offs_y[id], self.alt)

        while not self.cops[id].reached:
            self.cops[id].update(self.center_x + self.offs_x[id], self.center_y + self.offs_y[id], self.alt)
            print "not reached, x: ", self.cops[id].vx, " y: ", self.cops[id].vy, " alt: ",self.alt
            time.sleep(0.1)

        
if __name__ == '__main__':
    rospy.init_node("deep_teleop")
    pv = posVel()
    pv.start_subs()
    pv.subscribe_pose_thread()
    pv.start_navigating()
    pv.controller_sub()
    pv.start_recording()
    time.sleep(0.2)

    print "set mode"
    pv.setmode(custom_mode="OFFBOARD")
    #pv.arm()

    time.sleep(0.1)
    #pv.takeoff_velocity()

    print "Telop should be functional!!"

    while not rospy.is_shutdown():
        pass

    pv.image_data.close()
    
