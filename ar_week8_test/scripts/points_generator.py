#!/usr/bin/env python3
import rospy
import random
from ar_week8_test.msg import cubic_traj_params

def generate_points():
    # Define the publisher
    pub = rospy.Publisher('params', cubic_traj_params, queue_size=10)
    # Initialize the node
    rospy.init_node('generator', anonymous=True)
    rate = rospy.Rate(0.05)  # Set the rate to 0.05Hz (20 seconds)

    while not rospy.is_shutdown():
        # Generate random values within specified limits
        p0 = random.uniform(-10, 10)
        pf = random.uniform(-10, 10)
        v0 = random.uniform(-10, 10)
        vf = random.uniform(-10, 10)
        t0 = 0  # t0 is always 0
        dt = random.uniform(5, 10)
        tf = t0 + dt

        # Create a message and assign the generated values
        msg = cubic_traj_params(p0=p0, pf=pf, v0=v0, vf=vf, t0=t0, tf=tf)

        # Publish the message
        pub.publish(msg)
        rospy.loginfo("Published cubic_traj_params with p0: %f, pf: %f, v0: %f, vf: %f, t0: %f, tf: %f" % (p0, pf, v0, vf, t0, tf))

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        generate_points()
    except rospy.ROSInterruptException:
        pass
