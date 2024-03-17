# #!/usr/bin/env python3
# import rospy
# from ar_week8_test.msg import cubic_traj_coeffs
# from std_msgs.msg import Float32

# # Global publisher objects
# pub_position = None
# pub_velocity = None
# pub_acceleration = None

# def coeffs_callback(data):
#     rospy.loginfo("Received cubic trajectory coefficients")
#     t0 = data.t0
#     tf = data.tf
#     time_steps = tf - t0
    
#     # Time vector from t0 to tf (Here we're simplifying to use only the first value for demonstration)
#     t = t0  # Only using the initial time
    
#     # Calculate position, velocity, and acceleration for the first time point
#     position = data.a0 + data.a1*t + data.a2*t**2 + data.a3*t**3
#     velocity = data.a1 + 2*data.a2*t + 3*data.a3*t**2
#     acceleration = 2*data.a2 + 6*data.a3*t
    
#     # Publish the trajectories (only the first data point of each)
#     pub_position.publish(Float32(data=position))
#     pub_velocity.publish(Float32(data=velocity))
#     pub_acceleration.publish(Float32(data=acceleration))

# def plot_cubic_traj():
#     global pub_position, pub_velocity, pub_acceleration
    
#     rospy.init_node('plotter')
    
#     rospy.Subscriber("coeffs", cubic_traj_coeffs, coeffs_callback)
    
#     pub_position = rospy.Publisher('trajPos', Float32, queue_size=10)
#     pub_velocity = rospy.Publisher('trajVel', Float32, queue_size=10)
#     pub_acceleration = rospy.Publisher('trajAcc', Float32, queue_size=10)
    
#     rospy.spin()

# if __name__ == '__main__':
#     plot_cubic_traj()


#!/usr/bin/env python3
import rospy
from ar_week8_test.msg import cubic_traj_coeffs
from std_msgs.msg import Float32
import numpy as np

# Global publisher objects
pub_position = None
pub_velocity = None
pub_acceleration = None

def coeffs_callback(data):
    rospy.loginfo("Received cubic trajectory coefficients")
    t0 = data.t0
    tf = data.tf

    # Define a time step
    time_step = 0.0001  # This defines the resolution of your simulation
    
    # Generate the time points
    times = np.arange(t0, tf, time_step)
    
    for t in times:
        # Calculate position, velocity, and acceleration for each time point
        position = data.a0 + data.a1*t + data.a2*t**2 + data.a3*t**3
        velocity = data.a1 + 2*data.a2*t + 3*data.a3*t**2
        acceleration = 2*data.a2 + 6*data.a3*t

        # Publish the trajectories
        pub_position.publish(Float32(data=position))
        pub_velocity.publish(Float32(data=velocity))
        pub_acceleration.publish(Float32(data=acceleration))
        
        # Sleep to simulate real-time; adjust the sleep duration if necessary
        rospy.sleep(time_step)

def plot_cubic_traj():
    global pub_position, pub_velocity, pub_acceleration
    
    rospy.init_node('plotter')
    
    rospy.Subscriber("coeffs", cubic_traj_coeffs, coeffs_callback)
    
    pub_position = rospy.Publisher('trajPos', Float32, queue_size=10)
    pub_velocity = rospy.Publisher('trajVel', Float32, queue_size=10)
    pub_acceleration = rospy.Publisher('trajAcc', Float32, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    plot_cubic_traj()