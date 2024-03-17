#!/usr/bin/env python3
import rospy
from ar_week8_test.srv import compute_cubic_traj, compute_cubic_trajResponse
import numpy as np

def handle_compute_cubic_traj(req):
    rospy.loginfo("Received request for computing cubic trajectory coefficients")
    
    # Extract the request parameters
    p0 = req.p0
    pf = req.pf
    v0 = req.v0
    vf = req.vf
    t0 = req.t0
    tf = req.tf
    
    # Setup the matrix and vector for solving the system of equations
    # Matrix for the system of equations based on boundary conditions
    M = np.array([[1, t0, t0**2, t0**3],
                  [1, tf, tf**2, tf**3],
                  [0, 1, 2*t0, 3*t0**2],
                  [0, 1, 2*tf, 3*tf**2]])
    
    # Vector for the boundary conditions
    b = np.array([p0, pf, v0, vf])
    
    # Solve for the coefficients
    coeffs = np.linalg.solve(M, b)
    
    # Create the response message
    response = compute_cubic_trajResponse(*coeffs)
    
    return response

def compute_cubic_coeffs_server():
    rospy.init_node('computer')
    s = rospy.Service('compute_cubic_traj', compute_cubic_traj, handle_compute_cubic_traj)
    rospy.loginfo("Service compute_cubic_traj ready to compute trajectory coefficients.")
    rospy.spin()

if __name__ == "__main__":
    compute_cubic_coeffs_server()

