#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from find_object_2d.msg import ObjectsStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from PyQt5.QtCore import QPointF
from PyQt5.QtGui import QTransform
import os

music1_path = "/home/abot/mksw/src/detect2.wav"
music2_path = "/home/abot/mksw/src/arrive2.wav"
music3_path = "/home/abot/mksw/src/detect4.wav"
music4_path = "/home/abot/mksw/src/arrive4.wav"
music5_path = "/home/abot/mksw/src/detect5.wav"
music6_path = "/home/abot/mksw/src/arrive5.wav"
music7_path = "/home/abot/mksw/src/detect8.wav"
music8_path = "/home/abot/mksw/src/arrive8.wav"

path = [[music1_path,music2_path],[music3_path,music4_path],[music5_path,music6_path],[music7_path,music8_path]]

class navigation_demo:
    def __init__(self):
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
	self.objects_sub=rospy.Subscriber('/objects', Float32MultiArray,self.objects_cb)
	self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
	self.id=0
	self.cha=0
	self.save=[25]*15
	self.num=0
	self.cha_detected = False
        self.goal_reached = False  

    def ar_cb(self, data):
        self.qr_detected = False
        for ar_marker in data.markers:
            if ar_marker.id != 0 and ar_marker.id != 255:
        	self.qr_detected = True
                self.id=ar_marker.id
                print(id)

    def objects_cb(self, msg):
    	data = msg.data
	self.cha_detected = False
    	if data:
            for i in range(0, len(data), 12):
            	object_id = int(data[i])
            	object_width = data[i + 1]
            	object_height = data[i + 2]

            	qt_homography = QTransform(
                    data[i + 3], data[i + 4], data[i + 5],
                    data[i + 6], data[i + 7], data[i + 8],
                    data[i + 9], data[i + 10], data[i + 11]
            	)

            	qt_top_left = qt_homography.map(QPointF(0, 0))
            	qt_top_right = qt_homography.map(QPointF(object_width, 0))
            	qt_bottom_left = qt_homography.map(QPointF(0, object_height))
            	qt_bottom_right = qt_homography.map(QPointF(object_width, object_height))
		self.cha_detected = True
    		if ((55<=object_id) & (object_id<=60)):
	    	    print('a_1')
		    self.cha=1
    		if ((61<=object_id) & (object_id<=65)):
	            print('a_2')
		    self.cha=2
    		if ((66<=object_id) & (object_id<=70)):
	    	    print('a_3')
		    self.cha=3
    		if ((71<=object_id) & (object_id<=75)):
	    	    print('a_4')
		    self.cha=4
    		if ((76<=object_id) & (object_id<=81)):
	    	    print('a_5')
		    self.cha=5
    		if ((82<=object_id) & (object_id<=86)):
	    	    print('a_6')
		    self.cha=6
    		if ((87<=object_id) & (object_id<=91)):
	    	    print('a_7')
		    self.cha=7
    		if ((92<=object_id) & (object_id<=96)):
	    	    print('a_8')
		    self.cha=8
    
    def sway(self):
	while not rospy.is_shutdown() and not (self.cha_detected or  self.qr_detected):
		self.twist.angular.z=0.5
		self.cmd_vel_pub.publish(self.twist)
		rospy.sleep(1)
	self.twist.angular.z=0.0
	self.cmd_vel_pub.publish(self.twist)
	
    def revive(self):
        if self.goal_reached:
            self.twist.linear.x= 1.2
            self.twist.linear.y= 0.9
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(3)
        
        self.twist.linear.x= 0.0
        self.twist.linear.y= 0.0
        self.cmd_vel_pub.publish(self.twist)
        self.goal_reached=False

    def goto(self, p):
        rospy.loginfo("[Navi] goto %s" % p)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2] / 180.0 * pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
	    
    	while not self.move_base.wait_for_result(rospy.Duration(0.1)):
        	if (self.cha_detected or self.cha_detected):
		    if (self.id in self.save[:num]) or (self.cha in self.save[:num]):
			continue
            	    self.move_base.cancel_goal()
            	    rospy.loginfo("Navigation canceled due to detection.")
		    self.goal_reached = True
            	    return True 

    	state = self.move_base.get_state()
    	if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("reach goal %s succeeded!" % p)
	    self.goal_reached = True

        return True

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s" % (status, result))

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        msg = feedback
        #rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)


    def process_goal(self, p, targets):
        self.goto(p)
	if self.goal_reached==True:
	    if not (self.cha_detected or self.qr_detected):
		self.sway()
	    if (self.cha_detected or self.qr_detected):
	        os.system('mplayer %s' % path[num][0])
                rospy.sleep(2)
		    
	if self.goal_reached==False:
	    return True
		
        if self.num==0:
	    if(self.id==1 or self.cha==1):
		self.goto(targets[0])
                rospy.sleep(1)
	    if(self.id==2 or self.cha==2):
		self.save[num]=id
                self.goto(targets[1])
                rospy.sleep(1)
	        os.system('mplayer %s' % path[num][1])
	        self.goal_reached = False
	if self.num==1:
	    if(self.id==3 or self.cha==3):
		self.goto(targets[2])
                rospy.sleep(1)
	    if(self.id==4 or self.cha==4):
		self.save[num]=cha
                self.goto(targets[3])
                rospy.sleep(1)
	        os.system('mplayer %s' % path[num][1])
	        self.goal_reached = False
	if self.num==2:
	    if(self.id==6 or self.cha==6):
		self.goto(targets[5])
                rospy.sleep(1)
	    if(self.id==5 or self.cha==5):
		self.save[num]=id
                self.goto(targets[4])
                rospy.sleep(1)
	        os.system('mplayer %s' % path[num][1])
                self.goal_reached = False	    
	if self.num==3:
	    if(self.id==7 or self.cha==7):
		self.goto(targets[6])
                rospy.sleep(1)
	    if(self.id==8 or self.cha==8):
		self.save[num]=id
                self.goto(targets[7])
                rospy.sleep(1)
	        os.system('mplayer %s' % path[num][1])
                self.goal_reached = False

if __name__ == "__main__":
    rospy.init_node('navigation_demo', anonymous=True)

    goalListX = rospy.get_param('~goalListX', '2.0, 2.0')
    goalListY = rospy.get_param('~goalListY', '2.0, 4.0')
    goalListYaw = rospy.get_param('~goalListYaw', '0, 90.0')

    goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","), goalListY.split(","), goalListYaw.split(","))]
    print(goals)
    print('Please 1 to continue: ')
    targets=[[0.20,-1.02,-180],[1.00,-0.17,-180],[2.20,-0.17,0],[3.00,-1.02,0],[3.00,-2.22,0],[2.20,-3.07,0],[1.00,-3.07,-180],[0.20,-2.22,-180]]
    navi = navigation_demo()
    input = raw_input()
    if input == '1':
	for goal in goals:
            navi.process_goal(goal,targets)
	    navi.num=navi.num+1

    while not rospy.is_shutdown():
        rospy.sleep(1)
        
'''

 
    
	navi.goto([0.4,-3.20,0])
        navi.goto([0.2,-3.2,-180.0])
        rospy.sleep(2)
        navi.sway()
	navi.twist.linear.x= 1.0
        navi.twist.linear.y= 0.9
        navi.cmd_vel_pub.publish(navi.twist)
        rospy.sleep(3)
        
        navi.twist.linear.x= 0.0
        navi.twist.linear.y= 0.0
        navi.cmd_vel_pub.publish(navi.twist)

	
    if input == '2':
	navi.goto([0.3,-0.15,0])
	navi.goto([0.1,-0.1,0])
    if input == '3':
	navi.goto([0.3,-0.15,0])
	navi.goto([0.05,-0.05,0])
'''
