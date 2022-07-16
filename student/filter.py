# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    @property
    def F(self):
        ############
        return np.matrix([[1, 0, 0, params.dt, 0, 0],
                        [0, 1, 0, 0, params.dt, 0],
                        [0, 0, 1, 0, 0, params.dt],
                        [0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 1]])
        ############ 
        ############
        # END student code
        ############ 
    @property
    def Q(self):
        ############
        q3 = params.dt**3*params.q/3
        q2 = params.dt**2*params.q/2
        q1 = params.dt*params.q
        return np.matrix([[q3, 0, 0, q2, 0, 0],
                        [0, q3, 0, 0, q2, 0],
                        [0, 0, q3, 0, 0, q2],
                        [q2, 0, 0, q1, 0, 0],
                        [0, q2, 0, 0, q1, 0],
                        [0, 0, q2, 0, 0, q1]])
        ############
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        track.x = self.F*track.x # state prediction
        track.set_x(track.x)
        track.P = self.F*track.P*self.F.transpose() + self.Q # covariance prediction
        track.set_P(track.P)
        return track.x, track.P
        ############

        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        H = meas.sensor.get_H((track.x))
        gamma = self.gamma(track,meas)
        S = self.S(track, meas, None)
        K = track.P*H.transpose()*np.linalg.inv(S) # Kalman gain
        track.x = track.x + K*gamma # state update
        track.set_x(track.x)
        I = np.identity(params.dim_state)
        track.P = (I - K*H) * track.P # covariance update
        track.set_P(track.P)
        # return track.x, track.P   
        ############
        
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        return meas.z - meas.sensor.get_hx(track.x)
        ############
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        H = meas.sensor.get_H((track.x))
        return H*track.P*H.transpose() + meas.R
        ############
        ############
        # END student code
        ############ 