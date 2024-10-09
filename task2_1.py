#!/usr/bin/env python
import rospy
#from playsound import playsound
import actionlib
import time
import os
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi
from std_msgs.msg import String

from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
id=0  
flog=0

#z:音乐路径中不应该引号内再包含引号
"""
music1_path="/home/abot/abot_music/'music1.mp3'"
music2_path="/home/abot/abot_music/'music2.mp3'"
music3_path="/home/abot/abot_music/'music3.mp3'"
music4_path="/home/abot/abot_music/'music4.mp3'"
"""
music1_path = "/home/abot/abot_music/music1.mp3"
music2_path = "/home/abot/abot_music/music2.mp3"
music3_path = "/home/abot/abot_music/music3.mp3"
music4_path = "/home/abot/abot_music/music4.mp3"

class navigation_demo:
    def __init__(self):
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.arrive_pub = rospy.Publisher('/voiceWords',String,queue_size=10)
        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb);
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.amcl_pose_callback=rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        #self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)  
        #self.current_position = rospy.Subscriber('/amcl', PoseWithCovarianceStamped, self.amcl_pose_callback)  
        self.odom_data = None
        self.current_position_data = None
        #self.id=0
        #self.flog=0
	self.cmd_vel_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	self.twist=Twist()
	self.qr_detected=False

    def ar_cb(self, data):
        for ar_marker in data.markers:
          if ar_marker.id!=0 and ar_marker.id!=255:
              self.qr_detected = True
              print ar_marker.id

    def set_pose(self, p):
        if self.move_base is None:
            return False

        x, y, th = p

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, th/180.0*pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        self.set_pose_pub.publish(pose)
        return True

    def amcl_pose_callback(self,msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        rospy.loginfo("Current Position: x = %.2f, y = %.2f, z = %.2f", position.x, position.y, position.z)
        rospy.loginfo("Current Orientation: x = %.2f, y = %.2f, z = %.2f, w = %.2f", orientation.x, orientation.y, orientation.z, orientation.w)
        if position.x!=0.2 and position.y!=-0.65:
            self.goto([0.2,-0.65,-180])

    def sway(self):
        while not rospy.is_shutdown() and not self.qr_detected:
            self.twist.angular.z=-0.5
	    self.cmd_vel_pub.publish(self.twist)
	    rospy.sleep(0.5)
        self.twist.angular.z=0.0
        self.cmd_vel_pub.publish(self.twist)

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s"%(status, result))
        arrive_str = "arrived to traget point"
        self.arrive_pub.publish(arrive_str)

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        msg = feedback
        #rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)

    def goto(self, p):
        rospy.loginfo("[Navi] goto %s"%p)
        #arrive_str = "going to next point"
        #self.arrive_pub.publish(arrive_str)
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
	#z:修改一下逻辑，检查目标发送是否成功并沿用超时逻辑
	# 记录开始时间  
        start_time = rospy.Time.now()  

        while not rospy.is_shutdown():  
            # 检查目标状态  
            state = self.move_base.get_state()  

            if state == GoalStatus.REJECTED:  
                rospy.logwarn("Goal Rejected. Trying to republish.")  
                self.move_base.send_goal(goal)  # 重发目标  
                start_time = rospy.Time.now()  # 重置开始时间  

            elif state == GoalStatus.SUCCEEDED:  
                rospy.loginfo("Reached goal succeeded!")  
                break  

            # 检查是否超过60秒  
            if (rospy.Time.now() - start_time).to_sec() > 60:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
                break  

            # 可选：添加小睡眠以防止紧密循环  
            rospy.sleep(1) 
	    """
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p)
        return True
	    """
    def cancel(self):
        self.move_base.cancel_all_goals()
        return True

def find_id(id):
    if id==1:
        os.system('mplayer %s' % music1_path)
        print('music1')
    elif id==2:
        os.system('mplayer %s' % music2_path)
        print('music2')
    elif id==3:
        os.system('mplayer %s' % music3_path)
        print('music3')
    elif id==4:
        os.system('mplayer %s' % music4_path)
        print('music4')
    elif id==5:
        os.system('mplayer %s' % music1_path)
        print('music5')
    elif id==6:
        os.system('mplayer %s' % music2_path)
        print('music6')
    elif id==7:
        os.system('mplayer %s' % music3_path)
        print('music7')
    elif id==8:
        os.system('mplayer %s' % music4_path)
        print('music8')



if __name__ == "__main__":
    rospy.init_node('navigation_demo',anonymous=True)

    goalListX = rospy.get_param('~goalListX', '2.0, 2.0')
    goalListY = rospy.get_param('~goalListY', '2.0, 4.0')
    goalListYaw = rospy.get_param('~goalListYaw', '0, 90.0')

    goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","),goalListY.split(","),goalListYaw.split(","))]
    print ('Please 1 to continue: ')
    input = raw_input()
    print (goals)
    navi = navigation_demo()
    if input == '1':
	navi.goto(goals[0])
	time.sleep(1)
	'''if not navi.qr_detected:
		navi.sway()  '''  
