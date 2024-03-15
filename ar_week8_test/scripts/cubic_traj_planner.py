#!/usr/bin/env python
import rospy
from ar_week8_test.msg import cubic_traj_params, cubic_traj_coeffs
from ar_week8_test.srv import compute_cubic_traj

def callback(data):
    rospy.wait_for_service('compute_cubic_traj')
    try:
        compute_traj = rospy.ServiceProxy('compute_cubic_traj', compute_cubic_traj)
        resp = compute_traj(data.p0, data.pf, data.v0, data.vf, data.t0, data.tf)
        pub.publish(resp.a0, resp.a1, resp.a2, resp.a3, data.t0, data.tf)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def cubic_traj_planner():
    rospy.init_node('planner', anonymous=True)
    rospy.Subscriber("params", cubic_traj_params, callback)
    global pub
    pub = rospy.Publisher('coeffs', cubic_traj_coeffs, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    cubic_traj_planner()
