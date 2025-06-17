import numpy as np
from enum import Enum
from .search_pattern import (LinearSearchPattern,
                             SinusoidalSearchPattern,
                             CompositeSearchPattern)

from .state_machine import State

class SimpleDemoStateMachine(object):

    def __init__(self,
                 dt,
                 alpha_rate=1.0/10.0,
                 takeoff_position=np.array([0.0, 0.0, 1.5])):

        self.dt = dt
        self.alpha_rate = alpha_rate
        self.alpha = np.ones(3)
        self.state = State.TAKEOFF
        self.takeoff_position = takeoff_position

        self.target_pos_estimate = np.zeros(3)
        self.target_yaw_estimate = 0.0
        self.reference_pos = np.zeros(3)

    def reset(self):

        self.alpha = np.ones(3)
        self.state = State.TAKEOFF

        self.reference_pos = np.zeros(3)
        self.target_pos_estimate = np.zeros(3)

    def set_target_pos_estimate(self, pos, yaw):
        """Set the target position estimate."""
        self.target_pos_estimate = np.array(pos)
        self.target_yaw_estimate = yaw

    def takeoff_control(self, x, v, contact):
        """
        Control for takeoff state.
        This is a placeholder and should be implemented based on the specific requirements of the takeoff procedure.
        """
        yaw_des = self.target_yaw_estimate
        p_des = self.takeoff_position
        v_des = np.zeros(4)  # No velocity control during takeoff

        return {'alpha': self.alpha,
                'p_des': p_des,
                'v_des': v_des,
                'yaw_des': yaw_des}

    def searching_position_control(self, x, v, contact):

        # Explicitly state that we are not using this here
        _, _ = contact, v

        # Extract new desired control values
        p_des = self.target_pos_estimate - np.array([0.15, 0.15, 0.0]) - np.array([0, 0, 0.15])
        yaw_des = self.target_yaw_estimate

        # Return control actions
        return {'alpha': self.alpha,
                'p_des': p_des,
                'v_des': np.zeros(4),
                'yaw_des': yaw_des}

    def position_align_control(self, x, v, contact, p_des=None):
        
        p_des = self.target_pos_estimate - np.array([0, 0, 0.15])
        yaw_des = self.target_yaw_estimate

        v_des = np.zeros(4)        
        
        return {'alpha': self.alpha,
                'p_des': p_des,
                'v_des': v_des,
                'yaw_des': yaw_des}
        
    def perch_control(self, x, v, contact):

        p_des = self.target_pos_estimate - np.array([0, 0, 0.1])
        yaw_des = self.target_yaw_estimate

        dalpha = self.alpha_rate * self.dt * np.ones(3)
        self.alpha -= dalpha
        self.alpha = np.clip(self.alpha, 0.0, 1.0)
        v_des = np.zeros(4)
        
        return {'alpha': self.alpha,
                'p_des': p_des,
                'v_des': v_des,
                'yaw_des': yaw_des}
    
    def finalize_grasp_control(self, x, v, contact):
            
        p_des = self.target_pos_estimate
        yaw_des = self.target_yaw_estimate

        self.alpha = np.zeros(3)
        v_des = np.zeros(4)
        
        return {'alpha': self.alpha,
                'p_des': p_des,
                'v_des': v_des,
                'yaw_des': yaw_des}
    
    def control(self, x, v, contact):

        if self.state == State.TAKEOFF:
            ctrl = self.takeoff_control(x, v, contact)
            self.reference_pos = ctrl["p_des"]  
            if np.linalg.norm(x[:3] - self.reference_pos) < 0.1 and np.linalg.norm(v[:3]) < 0.05:
                self.state = State.SEARCHING
                print("STATE CHANGE: TAKEOFF -> SEARCHING")
        elif self.state == State.SEARCHING:
            ctrl = self.searching_position_control(x, v, contact)
            self.reference_pos = ctrl["p_des"]  
            if np.linalg.norm(x[:3] - self.reference_pos) < 0.1 and np.linalg.norm(v[:3]) < 0.05:
                self.state = State.TOUCHED
                print("STATE CHANGE: SEARCHING -> TOUCHED")
        elif self.state == State.TOUCHED:
            ctrl = self.position_align_control(x, v, contact)
            self.reference_pos = ctrl["p_des"]  
            if np.linalg.norm(x[:3] - self.reference_pos) < 0.1 and np.linalg.norm(v[:3]) < 0.05:
                self.state = State.ROTATION
                print("STATE CHANGE: TOUCHED -> ROTATION")
        elif self.state == State.ROTATION:
            ctrl = self.perch_control(x, v, contact)
            self.reference_pos = ctrl["p_des"]  
            if ((np.linalg.norm(x[:3] - self.reference_pos) < 0.1) and
               (self.alpha < 0.05).all()):
                self.state = State.FINALIZE
                print("STATE CHANGE: ROTATION -> FINALIZE")
        elif self.state == State.FINALIZE:
            ctrl = self.finalize_grasp_control(x, v, contact)
        else:
            ctrl = self.position_align_control(x, v, contact)
        return ctrl
