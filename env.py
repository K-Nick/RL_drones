import numpy as np
import scipy.linalg as la

class RigidBody(object):
    def __init__(self, state, m=1, radius, is_player):
        # current state
        # [x, y, z, vx, vy, vz, ax, ay, az]
        self.state = state
        self.m = m
        self.radius = radius
        self.is_player = is_player

    def applyForce(self, vec):
        self.state[6:9] = vec
 
    def stateTransitionEquation(self):
        state_delta = np.hstack([self.state[3:6], self.state[6:9]/self.m, np.zeros(3)])
        return state_delta
 
    def eulerIntegrate(self, state, state_delta, dt):
        #Integrate state changes
        next_state = state + state_delta * dt
        return next_state 
 
    def collideDetection(self):
        

    def update(self, dt):
        state_delta = self.stateTransitionEquation()
        next_state = self.eulerIntegrate(self.state, state_delta, dt)
        self.state = next_state

def Space(object):
    def __init__(self, h, w):
        obj_list = []
        self.h = h
        self.w = w

    def add_object(self, obj):
        obj_list.append(obj)
    
