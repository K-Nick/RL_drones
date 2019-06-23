import numpy as np
import scipy.linalg as la

EPS = 1e-4

def eularDistance(obj1, obj2):
    pos1 = obj1.state[0:3]
    pos2 = obj2.state[0:3]
    return np.sqrt(np.sum((pos1 - pos2) * (pos1 - pos2)))

def dotMul(vec1, vec2):
    return np.sum(vec1 * vec2)

def vecNorm(vec):
    return np.sqrt(np.sum(vec * vec))

class RigidBody(object):
    def __init__(self, state, m, radius, is_player=False):
        # current state
        # [x, y, z, vx, vy, vz, ax, ay, az]
        self.state = np.array(state)
        self.m = m
        self.radius = radius
        self.is_player = is_player
        self.crashed = False

    def applyForce(self, vec):
        self.state[6:9] = vec
 
    def stateTransitionEquation(self):
        state_delta = np.hstack([self.state[3:6], self.state[6:9]/self.m, np.zeros(3)])
        return state_delta
 
    def eulerIntegrate(self, state, state_delta, dt):
        #Integrate state changes
        next_state = state + state_delta * dt
        return next_state 
 
    def collisionDetection(self, obj_list, bound):
        if (self.is_player):
            #if this body is the player, crash
            for obj in obj_list:
                if (eularDistance(obj, self) < self.radius + EPS):
                    self.crashed = True
                    break
            if (self.state[0] < bound[0] + EPS or self.state[1] < bound[1] + EPS or self.state[2] < bound[2] + EPS):
                    self.crashed = True
        else:
            #if this body is an obstruction, bounce
            for obj in obj_list:
                if (obj == self):
                    continue
                if (eularDistance(obj, self) < self.radius + obj.radius + EPS):
                    delta_vec =  obj.state[0:3] - self.state[0:3]
                    project = dotMul(self.state[3:6], delta_vec) / vecNorm(delta_vec)
                    normalized_delta_vec = delta_vec / vecNorm(delta_vec)
                    self.state[3:6] -= normalized_delta_vec * 2 * project
            for i in range(3):
                if (self.state[i] > self.radius + EPS or self.state[i] < EPS):
                    self.state[3+i] = -self.state[3+i]

    def update(self, dt):
        state_delta = self.stateTransitionEquation()
        next_state = self.eulerIntegrate(self.state, state_delta, dt)
        self.state = next_state

class Space(object):
    def __init__(self, size):
        self.obj_list = []
        self.size = size

    def add_object(self, obj):
        self.obj_list.append(obj)

    def update(self, dt):
        for obj in self.obj_list:
            obj.collisionDetection(self.obj_list, self.size)
        for obj in self.obj_list:
            obj.update(dt)
    
if __name__ == "__main__":
    a = RigidBody([0.5,0.5,0.5,1,1,1,0,0,0],1,1)
    b = Space([4,5,6])
    b.add_object(a)
    dt = 0.01
    for i in range(100):
        print(i, b.obj_list[0].state)
        b.update(dt)