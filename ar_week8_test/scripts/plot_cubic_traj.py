#!/usr/bin/env python3
import rospy
from ar_week8_test.msg import cubic_traj_coeffs
import matplotlib.pyplot as plt
import numpy as np

def callback(data):
    t = np.linspace(data.t0, data.tf, 100)
    position = data.a0 + data.a1*t + data.a2*t**2 + data.a3*t**3
    velocity = data.a1 + 2*data.a2*t + 3*data.a3*t**2
    acceleration = 2*data.a2 + 6*data.a3*t
    
    plt.figure()
    plt.plot(t, position, label='Position')
    plt.plot(t, velocity, label='Velocity')
    plt.plot(t, acceleration, label='Acceleration')
    plt.legend()
    plt.show()

def plot_cubic_traj():
    rospy.init_node('plotter', anonymous=True)
    rospy.Subscriber("coeffs", cubic_traj_coeffs, callback)
    rospy.spin()

if __name__ == '__main__':
    plot_cubic_traj()
