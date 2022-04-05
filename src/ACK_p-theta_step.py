#!/usr/bin/env python

import rospy
import roslib
import sys
import time
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from scipy import linalg
import conversion_lib
from blob_detection_v6 import *

#Does this need to be globaled? I really hope not
d_camera_array = []

class AckPTheta:

    def __init__(self):

        # Get params from launch file
        points_array = rospy.get_param('~points_array', None)
        delay_time = rospy.get_param('~delay_time', 0)
        self.r = rospy.get_param('~r', 0.07)
        self.l = rospy.get_param('~l', 0.36)
        self.k_p = rospy.get_param('~k_p', 10)
        self.robot_d_sq = rospy.get_param('~threshold_dist', 1) ** 2
        self.target_vel = rospy.get_param('~target_vel', 1)
        self.w = self.target_vel
        init_state = rospy.get_param('~inital_state', [0, 0, 0])
        time.sleep(delay_time)
        print('started')
        self.start_time = rospy.Time.now().to_sec()

        # Set up waypoint array
        if points_array is None:
            # self.points = np.array([[0, 0], [7, -1], [1, 10], [8, 20]])
            self.points = np.array([[0, 0], [7, 0], [1, 10], [9, 20], [0, 20], [-7, 21], [-6, 10], [-7, -1], [3, -1]])
	    self.cv_color = np.array([['blue'], ['blue'], ['yellow'], ['blue'], ['blue'], ['blue'], ['red'], ['blue'], ['blue']]) #Double check this
        else:
            self.points = points_array
        self.current_point = 0
        self.loc = init_state[0:3]

        # Publishes the current [x, y, theta] state estimate
        self.angle_pub = rospy.Publisher("target_wheel_angle", Float32, queue_size=1)
        self.speed_pub = rospy.Publisher("target_velocity", Float32, queue_size=1)

        # Computes the current state estimate from the gps data
        self.ekf_sub = rospy.Subscriber("/EKF/Odometry", Odometry, self.callback)
        self.velocity_sub = rospy.Subscriber("/speed_current", Float32, self.velocity_callback)

    # Returns the angle difference between the current trajectory and the goal, measured CCW from the current trajectory
    def theta_error(self, x, y, t, x_d, y_d):
        t_goal = np.arctan2(y_d - y, x_d - x)
        e = t_goal - t
        ## CRITICAL: ENSURE THAT THE ERROR IS BETWEEN -PI, PI OTHERWISE IT BEHAVES WEIRD
        if e > np.pi:
            e = -np.pi * 2 + e
        elif e < -np.pi:
            e = np.pi * 2 + e
        return e

    def p_ik(self, e):
        return np.arctan2(self.r*self.l*self.k_p*e, self.w)

    def callback(self, data):
        if self.current_point < self.points.shape[0] - 1:
            print('target waypoint: ', self.points[self.current_point + 1])

            d = (self.points[self.current_point + 1][0] - self.loc[0]) ** 2 + \
                (self.points[self.current_point + 1][1] - self.loc[1]) ** 2

            print('distance', np.sqrt(d))
            ## Compute current position based on last time step and measurement
            # From EKF
            self.loc[0] = data.pose.pose.position.x
            self.loc[1] = data.pose.pose.position.y
            eul = conversion_lib.quat_from_pose2eul(data.pose.pose.orientation)
            self.loc[2] = eul[0]

            ## Compute the angle error
            e = self.theta_error(self.loc[0], self.loc[1], self.loc[2],
                                 self.points[self.current_point + 1][0], self.points[self.current_point + 1][1])
            #print('Angle error is: ' + str(e))
            # Compute new wheel angle ans send it to the car
            phi = self.p_ik(e)
            print('Angle sending to car: ' + str(phi))
            if phi < -0.5:
                 self.angle_pub.publish(-2)
            elif phi > 0.5:
                self.angle_pub.publish(2)
            else:
                self.angle_pub.publish(0)
            self.speed_pub.publish(self.target_vel)

            ## Determine if we passed the obstacle
            d = (self.points[self.current_point + 1][0] - self.loc[0]) ** 2 + \
                (self.points[self.current_point + 1][1] - self.loc[1]) ** 2

    	    #Distance Modify/average here!!!!
            d_camera = None
            cap = cv.VideoCapture(0)
            for i in range(0, 10):
                d_camera = detect(cap, 0.3, self.cv_color[self.current_point + 1])
                d_camera_array.append([d_camera, time.time()])

            if d_camera == None:
                vel = 0
                accel = 0
                vel_array = []
                if len(d_camera_array) < 11:
                  d_camera = (self.points[self.current_point + 1][0] - self.loc[0]) ** 2 + \
                             (self.points[self.current_point + 1][1] - self.loc[1]) ** 2
                else:
                  val_array = []
                  for k in range(0, len(d_camera_array)):
                    if (d_camera_array[len(d_camera_array) - k][0] != None) and (len(val_array) >= 11):
                      val_array.append(d_camera_array[len(d_camera_array) - k])
                  val_array_np = np.array(val_array)
                  dist_array = val_array_np[:,0]
                  time_array = val_array_np[:,1]

                  vel_array = []
                  accel_array = []
                  for i in range(0, len(val_array_np) - 1):
                    vel_array.append((dist_array[i + 1] - dist_array[i]) / (time_array[i + 1] - time_array[i]))

                    try:
                      accel_array.append((vel_array[i] - vel_array[i - 1]) / (time_array[i] - time_array[i - 1]))
                    except IndexError:
                      pass

                  x0 = val_array[len(val_array) - 1][0]
                  t = (0.1 * (1 + np.random.random() / 10))  #time.time() - val_array[len(val_array)][1]
                  vel_avg = np.average(vel_array)
                  accel_avg = np.average(accel_array)

                  d_camera = x0 + (vel_avg * t) + (0.5 * accel_avg * (t ** 2))

            d = (d + d_camera) / 2

            if d < self.robot_d_sq:
                self.current_point += 1
        else:
            self.speed_pub.publish(0.00)
            self.angle_pub.publish(0)

    def velocity_callback(self, data):
        # Update w from data
        self.w = data.data
        #print('Speed is: ' + str(self.w))

def main():
    rospy.init_node('ack_p_theta', anonymous=True)
    p_theta = AckPTheta()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
# current_point stores the index of the current point
# loc = [x, y, theta] current position
# while current_point < len(points):
    # if current_loc > len(points) - 1:
        # if v > (v_min + v_ramp):
            # v = v - v_ramp
    # elif v < (v_min - v_ramp):
        # v = v + v_ramp
    # e = theta_error()
    # w1, w2 = p_ik
    # loc = fk_dt
    # d = sqrt(points(current_point) - loc)
    # if d < robot_d:
        # current_point += 1
