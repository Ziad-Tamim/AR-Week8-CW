#!/usr/bin/env python3
import rospy
from ar_week8_test.srv import compute_cubic_traj

def compute_coefficients(req):
    # Extract request parameters
    p0, pf, v0, vf, t0, tf = req.p0, req.pf, req.v0, req.vf, req.t0, req.tf

    # Compute time delta
    delta_t = tf - t0

    # Compute coefficients based on the boundary conditions
    # These equations are derived from the boundary conditions of cubic polynomial trajectories
    a0 = p0
    a1 = v0
    a2 = (3*(pf - p0)/delta_t**2) - (2*v0/delta_t) - (vf/delta_t)
    a3 = (-2*(pf - p0)/delta_t**3) + ((v0 + vf)/delta_t**2)

    return ComputeCubicTrajResponse(a0, a1, a2, a3)

def compute_cubic_coeffs_server():
    rospy.init_node('computer')
    s = rospy.Service('compute_cubic_traj', compute_cubic_traj, compute_coefficients)
    rospy.loginfo("Ready to compute cubic coefficients.")
    rospy.spin()

if __name__ == "__main__":
    compute_cubic_coeffs_server()

