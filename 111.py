#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi
from std_msgs.msg import String

from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker

global id = 0

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
        self.arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.qr_detected = False
        self.goal_reached = False  

    def ar_cb(self, data):
        for ar_marker in data.markers:
            if ar_marker.id != 0 and ar_marker.id != 255：
                id=ar_marker.id
                print(id)

    def sway(self):
        while not rospy.is_shutdown() and not self.qr_detected:
            self.twist.angular.z = -1.0
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(0.5)
        
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

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
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
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

    def voice(id):
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

    def process_goal(self, p, targets):
        self.goto(p)
        self.voice(id)
        rospy.sleep(2)
        if self.goal_reached and id==1:
            self.goto[targets[0]]
            rospy.sleep(2)
            self.goto[targets[1]]
            rospy.sleep(2)
        if self.goal_reached and id==6:
            self.goto[targets[2]]
            rospy.sleep(2)
            self.goto[targets[3]]
            rospy.sleep(2)
        if self.goal_reached and id==5:
            self.goto[targets[4]]
            rospy.sleep(2)
            self.goto[targets[5]]
            rospy.sleep(2)
        if self.goal_reached and id==4:
            self.goto[targets[6]]
            rospy.sleep(2)
            self.goto[targets[7]]
            rospy.sleep(2)

        self.goal_reached = False


if __name__ == "__main__":
    rospy.init_node('navigation_demo', anonymous=True)

    goalListX = rospy.get_param('~goalListX', '2.0, 2.0')
    goalListY = rospy.get_param('~goalListY', '2.0, 4.0')
    goalListYaw = rospy.get_param('~goalListYaw', '0, 90.0')

    goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","), goalListY.split(","), goalListYaw.split(","))]
    print('Please 1 to continue: ')
    targets=[[0.20,-1.02],[1.00,-0.17],[2.20,-0.17],[3.00,-1.02],[3.00,-2.22],[2.20,-3.07],[1.00,-3.07],[0.20,-2.22]]
    
    input = raw_input()
    print(goals)

    navi = navigation_demo()
    if input == '1':
        for goal in goals:
            navi.process_goal(goal,targets)

    while not rospy.is_shutdown():
        rospy.sleep(1)
        
'''if self.goal_reached and not self.qr_detected:
            self.sway()
            self.qr_detected = False'''
