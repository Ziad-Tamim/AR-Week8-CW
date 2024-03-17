#!/usr/bin/env python3
import rospy
from ar_week8_test.msg import cubic_traj_params, cubic_traj_coeffs
from ar_week8_test.srv import compute_cubic_traj

def params_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " Received trajectory params: %s", data)
    try:
        # Wait for the compute_cubic_traj service to become available
        rospy.wait_for_service('compute_cubic_traj')
        
        # Create a service proxy
        compute_traj = rospy.ServiceProxy('compute_cubic_traj', compute_cubic_traj)
        
        # Call the service
        resp = compute_traj(data.p0, data.pf, data.v0, data.vf, data.t0, data.tf)
        
        # Publish the computed trajectory coefficients
        coeffs = cubic_traj_coeffs()
        coeffs.a0 = resp.a0
        coeffs.a1 = resp.a1
        coeffs.a2 = resp.a2
        coeffs.a3 = resp.a3
        coeffs.t0 = data.t0
        coeffs.tf = data.tf
        
        pub.publish(coeffs)
        rospy.loginfo(f"Published cubic trajectory coeffs: {coeffs}")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def cubic_traj_planner():
    # Initialize the ROS node
    rospy.init_node('planner')
    
    # Create a subscriber to listen to the cubic_traj_params_topic
    rospy.Subscriber("params", cubic_traj_params, params_callback)
    
    # Define the publisher globally
    global pub
    pub = rospy.Publisher('coeffs', cubic_traj_coeffs, queue_size=10)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    cubic_traj_planner()
