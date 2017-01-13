#!/usr/bin/env python
import rospy
import numpy as np


class Vehicle:
    # Vehicle parameters
    param = {'m':    0.0,  # Mass in Kg
             'l':    0.0,  # Wheelbase in m
             'a':    0.0,  # Distance from cg to front axle in m
             'Iz':   0.0,  # Yaw moment of inertial in Kg m^2
             'Cf':   0.0,  # Front axle cornering stiffness in N/rad
             'Cr':   0.0,  # Rear axle cornering stiffness in N/rad
             'maxb': 0.0}  # Maximum brake acceleration in m/s^2

    # Vehicle state in [m, m, rad, m/s, m/s, rad/s]
    x = np.array([[0, 0, 0, 0, 0, 0]]).T

    # Simulation time in s
    t = 0

    # Load vehicle params and initial state from ROS
    def __init__(self):
        self.param['m'] = rospy.get_param('~parameters/m',
                                          self.param['m'])
        self.param['l'] = rospy.get_param('~parameters/l', self.param['l'])
        self.param['a'] = rospy.get_param('~parameters/a', self.param['a'])
        self.param['Iz'] = rospy.get_param('~parameters/Iz', self.param['Iz'])
        self.param['Cf'] = rospy.get_param('~parameters/Cf', self.param['Cf'])
        self.param['Cr'] = rospy.get_param('~parameters/Cr', self.param['Cr'])
        self.param['maxb'] = rospy.get_param('~parameters/maxb',
                                             self.param['maxb'])
        self.x[0] = rospy.get_param('~initial_state/x',
                                    self.x[0])
        self.x[1] = rospy.get_param('~initial_state/y',
                                    self.x[1])
        self.x[2] = rospy.get_param('~initial_state/yaw',
                                    self.x[2])
        self.x[3] = rospy.get_param('~initial_state/vx',
                                    self.x[3])
        self.x[4] = rospy.get_param('~initial_state/vy',
                                    self.x[4])
        self.x[5] = rospy.get_param('~initial_state/r',
                                    self.x[5])

    # Numerical integrator function that returns state
    def step(self, u, dt, substeps=1):
        dt = dt / substeps
        for i in range(substeps):
            self.x = self.x + dt * self.df(u)
        return self.x

    # Vehicles dynamics based on nonlinear chassis with linear tires
    def df(self, u):
        # Auxiliary variables for readibility
        yaw = self.x[2]
        vx = self.x[3]
        vy = self.x[4]
        r = self.x[5]

        # Inverse values for readibility
        m_inv = 1.0 / self.param['m']
        iz_inv = 1.0 / self.param['Iz']

        # Disntace from CG to rear axle
        b = (self.param['l'] - self.param['a'])

        # Control inputs for readibility
        axf = u[0]
        delta = u[1]

        # Front tire longitudinal force in N
        Fxf = axf * self.param['m']
        # Front tire lateral sliop in rad
        slipf = np.arctan2(vy + self.param['a'] * r, vx) - delta
        # Front tire lateral force in N
        Fyf = - self.param['Cf'] * slipf

        # Rear tire lateral sliop in rad
        slipr = np.arctan2(vy - b * r, vx)
        # Rear tire lateral force in N
        Fyr = - self.param['Cr'] * slipr

        # System equations from Dynamic Bicycle Model
        dx = np.zeros((6, 1))
        dx[0] = vx * np.cos(yaw) - vy * np.sin(yaw)
        dx[1] = vx * np.sin(yaw) + vy * np.cos(yaw)
        dx[2] = r
        dx[3] = (Fxf * np.cos(delta) - Fyf * np.sin(delta)) * m_inv + vy * r
        dx[4] = (Fxf * np.sin(delta) + Fyf * np.cos(delta) + Fyr) * m_inv - \
            vx * r
        dx[5] = (self.param['a'] * Fyf * np.cos(delta) - b * Fyr) * iz_inv

        print axf, Fxf, dx[3]
        return dx
