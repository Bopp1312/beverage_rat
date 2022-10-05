#!/usr/bin/python3

import rospy
import numpy as np
import threading
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

class Kalman_node:
    def __init__(self):
        rospy.Subscriber("/imu", Imu, self.imu_callback, queue_size=1)
        rospy.Subscriber("/odom", Twist, self.odom_callback, queue_size=1)
        rospy.Subscriber("/visual", Pose, self.visual_callback, queue_size=1)
        rospy.Subscriber("/cmd/velocity", Twist, self.command_callback, queue_size=1)
        self.pose_pub = rospy.Publisher("/kalman/pose", Pose, queue_size=1)

        # Create uncertainty matrices
        self.P_0 = np.diag((0.001, 0.001, 0.001, 0.001, 0.001, 0.001))
        Q = self.P_0
        R = np.diag((0.1, 0.1, 0.001, 0.001))

        self.pose = Pose()

        freq = 30
        dt = 1/freq

        # Initialize states
        self.theta = 0
        self.theta_dot = 0

        self.X_0 = np.matrix([[0],
                             [0],
                             [0],
                             [0],
                             [0],
                             [0]])
        self.A = np.matrix([[1, 0, 0, dt,  0,  0],
                            [0, 1, 0,  0, dt,  0],
                            [0, 0, 1,  0,  0, dt],
                            [0, 0, 0,  0,  0,  0],
                            [0, 0, 0,  0,  0,  0],
                            [0, 0, 0,  0,  0,  0]])

        self.H = np.matrix([[0,0,0,1,0,0],
                            [0,0,0,0,1,0],
                            [0,0,1,0,0,0],
                            [0,0,0,0,0,1]])

        self.X = self.X_0
        self.X_n_n = self.X_0
        self.P_n_n = self.P_0
        self.Q = Q
        self.R = R

        self.rate = rospy.Rate(freq)
        # Start loop on a thread

        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    def imu_callback(self, data):
        print("imu")
        quat = data.orientation
        rotation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        euler_angles = rotation.as_euler('xyz')
        self.theta = euler_angles[2]
        self.theta_dot = data.angular_velocity.z

    def odom_callback(self, data):
        print("odom")
        self.robot_lin_vel_x = data.linear.x
        self.robot_ang_vel_z = data.angular.z
        self.world_lin_vel_x = np.cos(self.theta)*self.robot_lin_vel_x
        self.world_lin_vel_y = np.sin(self.theta)*self.robot_lin_vel_x

    def visual_callback(self, data):
        # Use the visual pose estimate to
        # initialize the Kalman filter with the pose estimate
        print("visual_update")

    def command_callback(self, data):
        print("cmd")
        self.robot_lin_vel_x_cmd = data.linear.x
        self.robot_ang_vel_z_cmd = data.angular.z

    def update_estimation(self):
        print("State estimation")

        self.B = np.matrix([[0                 , 0],
                            [0                 , 0],
                            [0                 , 0],
                            [np.cos(self.theta), 0],
                            [np.sin(self.theta), 0],
                            [0                 , 1]])

        # Update control signal
        self.U = np.matrix([[self.robot_lin_vel_x_cmd],
                       [self.robot_ang_vel_z_cmd]])  

        # Simulate dynamics
        X_g = self.A*self.X+self.B*self.U
        Y_n = self.H*self.X

        # Populate Z vector with measurements
        z = np.matrix([[self.world_lin_vel_x],
                       [self.world_lin_vel_y],
                       [self.theta],
                       [self.theta_dot]])

        # State Estimation
        (X_e,P_1) = self.predict(self.A,self.B,self.U)
        self.update(self.H, z)

        print(X_e)
        self.pose.position.x = X_e[0,0]
        self.pose.position.y = X_e[1,0]
        self.pose_pub.publish(self.pose)

        self.X = X_g

    def loop(self):
        while(rospy.is_shutdown() == False):
            print("loop")
            try:
                self.update_estimation()
            except Exception as e:
                print("Problem: " + str(e))
            self.rate.sleep()

    def predict(self, A, B, U):
        X_1_n = A*self.X_n_n+B*U
        P_1_n = A*self.P_n_n*A.transpose()+self.Q
        self.P_1_n = P_1_n
        self.X_1_n = X_1_n
        return (X_1_n, P_1_n)

    def update(self, H, z_n):
        # Advance a time step
        self.X_n_n = self.X_1_n
        self.P_n_n = self.P_1_n
        X = self.X_n_n
        P = self.P_n_n
        R = self.R
        I = np.eye(6)

        K_n = P*H.transpose()*np.linalg.inv(H*P*H.transpose()+R)
        self.X_n_n = X + K_n*(z_n - H*X)
        self.P_n_n = (I-K_n*H)*P*(I-K_n*H).transpose()+K_n*R*K_n.transpose()
        return (self.X_n_n, self.P_n_n)


def main():
    rospy.init_node("kalman_filter", anonymous=True)
    obj = Kalman_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

if __name__ == "__main__":
    main()