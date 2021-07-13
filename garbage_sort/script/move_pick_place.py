#!/usr/bin/env python

import copy
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped


class MovePickPlace(object):
    def __init__(self):
        rospy.init_node('move_pick_place', anonymous=True)
        self.wheel_pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.pick_place_pub = rospy.Publisher('/pick_place', String, queue_size=10)
    
    def move(self, target_pose):
        """
        target_pose: geometry_msgs/PoseStamped
        """
        self.wheel_pose_pub.publish(target_pose)
        rospy.loginfo('move to target: {}'.format(target_pose))
    
    def pick(self):
        self.pick_place_pub.publish('pick')
        rospy.loginfo('pick garbage')
    
    def place(self):
        self.pick_place_pub.publish('place')
        rospy.loginfo('place garbage')
    
    def move_pick_place(self):
        # move -> pick -> move -> place
        pose_1 = PoseStamped()
        pose_1.header.frame_id = 'map'
        pose_1.pose.position.x = 0
        pose_1.pose.position.y = 0
        pose_1.pose.position.z = 0
        pose_1.pose.orientation.w = 0
        self.move(pose_1)
        rospy.sleep(10)
        self.pick()
        rospy.sleep(5)
        pose_2 = copy.deepcopy(pose_1)
        pose_2.pose.position.x += 1
        self.move(pose_2)
        rospy.sleep(10)
        self.place()
        rospy.sleep(5)
    

def main():
    mpp = MovePickPlace()
    mpp.move_pick_place()
 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass