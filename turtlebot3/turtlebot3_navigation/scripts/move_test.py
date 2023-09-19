import numpy as np
import rospy
import threading
import cmd, sys, os

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Twist
import actionlib

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class ControlSuiteShell(cmd.Cmd):
    intro = bcolors.OKBLUE + "Welcome to the control suite shell.\nType help or ? to list commands.\n" + bcolors.ENDC
    prompt = "(csuite) "

    def __init__(self):
        cmd.Cmd.__init__(self)
        rospy.init_node('simple_move_scenarios')
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    def do_move(self, arg):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = 0
        goal.target_pose.pose.position.y = -2
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = -1
        goal.target_pose.pose.orientation.w = 0.0

        self.translation_filtered = []
        self.translation_window = []
        self.quaternion_window = []
        self.client.send_goal(goal)
    
    def do_quit(self, arg):
        return True

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()