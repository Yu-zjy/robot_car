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

id = 0
flog = 0

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
            if ar_marker.id != 0 and ar_marker.id != 255 and self.goal_reached ==True:
                self.qr_detected = True
                print(ar_marker.id)

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
        arrive_str = "arrived to target point"
        self.arrive_pub.publish(arrive_str)

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        msg = feedback
        #rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)

    def process_goal(self, p):
        self.goto(p)
        rospy.sleep(2)

        if self.goal_reached and not self.qr_detected:
            self.sway()

        self.goal_reached = False
        self.qr_detected = False

if __name__ == "__main__":
    rospy.init_node('navigation_demo', anonymous=True)

    goalListX = rospy.get_param('~goalListX', '2.0, 2.0')
    goalListY = rospy.get_param('~goalListY', '2.0, 4.0')
    goalListYaw = rospy.get_param('~goalListYaw', '0, 90.0')

    goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","), goalListY.split(","), goalListYaw.split(","))]
    print('Please 1 to continue: ')
    input = raw_input()
    print(goals)

    navi = navigation_demo()
    if input == '1':
        for goal in goals:
            navi.process_goal(goal)

    while not rospy.is_shutdown():
        rospy.sleep(1)