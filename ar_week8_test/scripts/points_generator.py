#!/usr/bin/env python3
import rospy
from ar_week8_test.msg import cubic_traj_params
import random

def points_generator():
    # Initialize the ROS node
    rospy.init_node('generator') # , anonymous=True
    
    # Create a publisher object
    pub = rospy.Publisher('params', cubic_traj_params, queue_size=10)
    
    # Set the loop rate (how often to publish data)
    rate = rospy.Rate(0.05)  # Once every 20 seconds

    while not rospy.is_shutdown():
        # Generate random values within the specified ranges
        p0 = random.uniform(-10, 10)
        pf = random.uniform(-10, 10)
        v0 = random.uniform(-10, 10)
        vf = random.uniform(-10, 10)
        t0 = 0  # Initial time is always 0
        tf = t0 + random.uniform(5, 10)  # Final time is t0 + a random value between 5 and 10
        
        # Create a message object and assign the random values
        msg = cubic_traj_params()
        msg.p0 = p0
        msg.pf = pf
        msg.v0 = v0
        msg.vf = vf
        msg.t0 = t0
        msg.tf = tf
        
        # Log the information
        rospy.loginfo(f"Publishing: p0={p0}, pf={pf}, v0={v0}, vf={vf}, t0={t0}, tf={tf}")
        
        # Publish the message
        pub.publish(msg)
        
        # Sleep until the next cycle
        rate.sleep()

if __name__ == '__main__':
    try:
        points_generator()
    except rospy.ROSInterruptException:
        pass
