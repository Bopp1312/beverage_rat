#!/usr/bin/python3

import numpy as np
import time
import matplotlib.pyplot as plt

class kalman():
    def __init__(self, X_0, P_0, Q, R):
        self.X_n_n = X_0
        self.P_n_n = P_0
        self.Q = Q
        self.R = R

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

def loop():
    simulation_length = 5.0
    # Generate state input for u_w, u_phi

    r = 30/1000 # Wheel radius in Meters 
    l = 400/1000 # Base width in Meters

    u_w = 1.0
    u_phi = 2*np.pi/5
    U = np.matrix([[u_w],
                   [u_phi]])

    # Set Inital conditions
    x = 0
    y = 0
    theta = 0
    x_dot = 0
    y_dot = 0
    theta_dot = 0

    # Define State transition matrix
    freq = 30
    dt = 1/freq
    A = np.matrix([[1, 0, 0, dt,  0,  0],
                   [0, 1, 0,  0, dt,  0],
                   [0, 0, 1,  0,  0, dt],
                   [0, 0, 0,  0,  0,  0],
                   [0, 0, 0,  0,  0,  0],
                   [0, 0, 0,  0,  0,  0]])

    X = np.matrix([[x],
                   [y],
                   [theta],
                   [x_dot],
                   [y_dot],
                   [theta_dot]])

    H = np.matrix([[0,0,0,1,0,0],
                   [0,0,0,0,1,0],
                   [0,0,1,0,0,0],
                   [0,0,0,0,0,1]])
    
    X_0 = X

    # Create uncertainty matrices
    P_0 = np.diag((0.001, 0.001, 0.001, 0.001, 0.001, 0.001))
    Q = P_0
    R = np.diag((0.1, 0.1, 0.001, 0.001))

    # Create Kalman filter
    Kalman = kalman(X_0,P_0,Q,R)
    t = 0

    while True:
        # Update time
        t_n = t + dt
        
        # Exit criteria
        if t_n > simulation_length:
            break

        # Calculate percent of completion of sim
        percent = (t_n/simulation_length)*100.0

        # Update B matrix since its non-linear
        B = np.matrix([[0             , 0],
                       [0             , 0],
                       [0             , 0],
                       [np.cos(X[2,0]), 0],
                       [np.sin(X[2,0]), 0],
                       [             0, 1]])

        # Update control signal
        U = np.matrix([[u_w],
                       [u_phi]])    

        # Simulate dynamics
        X_g = A*X+B*U
        Y_n = H*X
        z = np.matrix([[X_g[0,0]],
                       [X_g[1,0]],
                       [X_g[2,0]],
                       [X_g[5,0]]])
        e = np.random.rand(4,1)

        # State Estimation
        (X_e,P_1) = Kalman.predict(A,B,U)
        Kalman.update(H, z+e)

        print("Time: " + str(t_n))
        print("States: " + str(X_g))
        print("Observations: " + str(Y_n))
        color_g = np.array([[0.1,0,percent/100.0]])
        color_e = np.array([[0.1,percent/100.0,0]])
        plt.scatter(X_g[0,0],X_g[1,0],c=color_g)
        plt.scatter(X_e[0,0],X_e[1,0],c=color_e)

        X = X_g
        t = t_n



if __name__ == "__main__":
    loop()
    print("Finished with simulation!")
    plt.show()