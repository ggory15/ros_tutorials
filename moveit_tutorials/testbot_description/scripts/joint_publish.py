#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

def joint_publish():
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rospy.init_node('joint_pub', anonymous=True)
    rate = rospy.Rate(100) # 100hz

    joint_state = JointState()
    joint_state.name.append("joint1")
    joint_state.name.append("joint2")
    joint_state.name.append("joint3")
    joint_state.position.append(0.0)
    joint_state.position.append(0.0)
    joint_state.position.append(0.0)

    while not rospy.is_shutdown():
        joint_state.header.stamp = rospy.Time.now()
        joint_state.position[0] = 1.0
        joint_state.position[1] = 2.0
        joint_state.position[2] = 1.0      

        print (joint_state)
        
        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_publish()
    except rospy.ROSInterruptException:
        pass
