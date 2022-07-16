# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
from scipy.stats.distributions import chi2

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params 

class Association:
    '''Data association class with single nearest neighbor association and gating based on Mahalanobis distance'''
    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []
        
    def associate(self, track_list, meas_list, KF):
             
        ############
        # TODO Step 3: association:
        # - replace association_matrix with the actual association matrix based on Mahalanobis distance (see below) for all tracks and all measurements
        # - update list of unassigned measurements and unassigned tracks
        ############
        
        # the following only works for at most one track and one measurement
        # self.association_matrix = np.matrix([]) # reset matrix
        # self.unassigned_tracks = [] # reset lists
        # self.unassigned_meas = []

        N = len(track_list) # N tracks
        M = len(meas_list) # M measurements
        self.unassigned_tracks = list(range(N))
        self.unassigned_meas = list(range(M))
        self.association_matrix = np.inf*np.ones((N,M)) 

        for n, track in enumerate(track_list):
            for m, meas in enumerate(meas_list):
                mhd = self.MHD(track=track, meas=meas, KF=KF)
                if self.gating(mhd, meas.sensor):
                    self.association_matrix[n,m] = mhd
        
        # if len(meas_list) > 0:
        #     self.unassigned_meas = [0]
        # if len(track_list) > 0:
        #     self.unassigned_tracks = [0]
        # if len(meas_list) > 0 and len(track_list) > 0: 
        #     self.association_matrix = np.matrix([[0]])
        
        ############
        # END student code
        ############ 
                
    def get_closest_track_and_meas(self):
        ############
        # - find indices of closest track and measurement for next update
        min_idx = np.unravel_index(np.argmin(self.association_matrix, axis=None), self.association_matrix.shape)
        # - return NAN if no more associations can be found (i.e. minimum entry in association matrix is infinity)
        if self.association_matrix[min_idx] == np.inf:
            return np.nan, np.nan 
        # - delete row and column in association matrix for closest track and measurement
        self.association_matrix = np.delete(self.association_matrix, min_idx[0],0)
        self.association_matrix = np.delete(self.association_matrix, min_idx[1],1)
        # - remove found track number from unassigned_tracks, meas number from unassigned_meas
        update_track = self.unassigned_tracks[min_idx[0]] 
        update_meas = self.unassigned_meas[min_idx[1]]
        self.unassigned_tracks.pop(min_idx[0])
        self.unassigned_meas.pop(min_idx[1])
        # - return indices of closest track and measurement for next update
        return update_track, update_meas
        ############

        # # the following only works for at most one track and one measurement
        # update_track = 0
        # update_meas = 0
        
        # # remove from list
        # self.unassigned_tracks.remove(update_track) 
        # self.unassigned_meas.remove(update_meas)
        # self.association_matrix = np.matrix([])
            
        # ############
        # # END student code
        # ############ 
        # return update_track, update_meas     

    def gating(self, MHD, sensor): 
        ############
        return MHD < chi2.ppf(params.gating_threshold, sensor.dim_meas)
        ############

        
        ############
        # END student code
        ############ 
        
    def MHD(self, track, meas, KF):
        ############
        gamma = KF.gamma(track,meas)
        S = KF.S(track, meas, None)
        MHD = gamma.transpose()*np.linalg.inv(S)*gamma # Mahalanobis distance formula
        return MHD
        ############
        
        
        ############
        # END student code
        ############ 
    
    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)
    
        # update associated tracks with measurements
        while self.association_matrix.shape[0]>0 and self.association_matrix.shape[1]>0:
            
            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print('---no more associations---')
                break
            track = manager.track_list[ind_track]
            
            # check visibility, only update tracks in fov    
            if not meas_list[0].sensor.in_fov(track.x):
                continue
            
            # Kalman update
            print('update track', track.id, 'with', meas_list[ind_meas].sensor.name, 'measurement', ind_meas)
            KF.update(track, meas_list[ind_meas])
            
            # update score and track state 
            manager.handle_updated_track(track)
            
            # save updated track
            manager.track_list[ind_track] = track
            
        # run track management 
        manager.manage_tracks(self.unassigned_tracks, self.unassigned_meas, meas_list)
        
        for track in manager.track_list:            
            print('track', track.id, 'score =', track.score)