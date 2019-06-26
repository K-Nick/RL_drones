from vpython import *
from vpython.vector import mag, mag2
import os
import numpy as np


INF = 99999999999.0

class Space(object):
    def __init__(self, ):
        # set parameters
        win = 700
        self.Natoms = 15
        self.N_ACTIONS = 27


        L = 1
        gray = color.gray(0.7)
        self.mass = 1
        self.Ratom = 0.03
        self.RO = 0.06  #radius of obstructions
        self.level = 0
        self.dist = [0.6, 0.4, 0.2, 0.1, 0.01]
        self.reward = [100, 200, 300, 600, 1000]
        self.init_pos = vector(0.45, 0.45, 0.45)
        self.target_pos = vector(-0.45, -0.45, -0.45)
        self.gameOver = False
        self.action_dict = {}

        #create action_dict
        cnt = 0
        for i = [-1, 0, 1]:
            for j = [-1, 0, 1]:
                for k = [-1, 0, 1]:
                    if (i==0 and j==0 and k==0): continue
                    action_dict[cnt] = vector(i, j, k)/mag(vector(i, j, k))
                    cnt += 1

        #create canvas
        #scene = display(visible = False)
        animation = canvas(width = win, height = win)
        animation.range = L
        animation.title = "drone simulation"
        
        #create cube(not solid)
        d = L/2 + self.Ratom
        r = 0.005
        boxbottom = curve([vector(-d,-d,-d), 
                            vector(-d,-d,d), 
                            vector(d,-d,d), 
                            vector(d,-d,-d), 
                            vector(-d,-d,-d)], color=gray, radius=r)
        boxtop = curve([vector(-d, d,-d), 
                            vector(-d, d,d), 
                            vector(d, d,d), 
                            vector(d, d,-d), 
                            vector(-d, d,-d)], color=gray, radius=r)
        vect = []
        vect.append(curve([vector(-d,-d,-d), vector(-d,d,-d)], color=gray, radius=r))
        vect.append(curve([vector(-d,-d,d), vector(-d,d,d)], color=gray, radius=r))
        vect.append(curve([vector(d,-d,d), vector(d,d,d)], color=gray, radius=r))
        vect.append(curve([vector(d,-d,-d), vector(d,d,-d)], color=gray, radius=r))

        #create player
        self.player = sphere(pos=self.init_pos, radius=self.Ratom, color=color.cyan)
        self.player.velocity = vector(-0.1, -0.1, -0.1)
        self.target = sphere(pos=self.target_pos, radius=1e-2, color=color.red)

        #create obstructions
        self.obstructions = []
        self.obs_pos = []
        self.p = []
        mass = 1

        for i in range(self.Natoms):
            x = L*random()-L/2
            y = L*random()-L/2
            z = L*random()-L/2
            cur_obj = sphere(pos = vector(x, y, z), radius=self.Ratom*2, color=color.yellow)
            self.obstructions.append(cur_obj)
            self.obs_pos.append(vector(x, y, z))

            # m = random()
            # mass.append(m)

            pavg = random() * mass
            theta = np.pi*random()
            phi = 2*np.pi*random()
            px = pavg*sin(theta)*cos(phi)
            py = pavg*sin(theta)*sin(phi)
            pz = pavg*cos(theta)
            self.p.append(vector(px, py, pz))

    def reset(self):
        self.__init__()
    
    def checkReward(self):
        cur_threshold = self.dist[self.level]
        player_pos = self.player.pos
        pos_delta = self.target_pos - player_pos
        ret = 0
        if (mag2(pos_delta) < cur_threshold):
            print("Get level ",self.level)
            ret = self.reward[self.level]
            self.level += 1
            if (self.level == 5):
                print("\nYou WIN!\n")
                self.gameOver = True
        return ret
    
    def checkGameOver(self):
        #check collision between obstruction and player
        d2 = (self.Ratom + self.RO) **2
        player_pos = self.player.pos
        for i in range(self.Natoms):
            pos = self.obs_pos[i]
            pos_delta = pos - player_pos
            if (mag2(pos_delta) < d2):
                print(pos_delta, self.obstructions[i].pos, player_pos)
                return True
        loc = self.player.pos
        if (abs(loc.x) > 1/2 or abs(loc.y) > 1/2 or abs(loc.z) > 1/2): return True
        return False


    def checkCollisions(self):
        #check collision between atoms
        hitlist = []
        d2 = (self.RO * 2) ** 2
        for i in range(self.Natoms):
            posx = self.obs_pos[i]
            for j in range(i):
                posy = self.obs_pos[j]
                pos_delta = posy - posx
                if (mag2(pos_delta) < d2):
                    hitlist.append((i,j))
        
        return hitlist

    def get_soner_arm(self, vec, pos_soner, pos_ball):
        vec = vec / mag(vec)
        pos_delta = pos_ball - pos_soner
        cos_side = dot(vec, pos_delta)
        dist2 = mag2(pos_delta) - cos_side**2
        if dist2 > self.RO ** 2:
            return INF
        arm = cos_side - np.sqrt(self.RO ** 2 - dist2)
        return arm
    
    def bound_soner_arm(self, vec):
        k = INF
        player_pos = self.player.pos
        arm_x = INF
        arm_y = INF
        arm_z = INF
        if(vec.x != 0):
            arm_x = np.maximum((0.5 - player_pos.x) / vec.x, (player_pos.x + 0.5) / vec.x)
        if(vec.y != 0):
            arm_y = np.maximum((0.5 - player_pos.y) / vec.y, (player_pos.y + 0.5)/ vec.y)
        if(vec.z != 0):
            arm_z = np.maximum((0.5 - player_pos.z) / vec.z, (player_pos.z + 0.5)/ vec.z)
        k = np.min([arm_x, arm_y, arm_z])
        return mag(vec) * np.abs(k)


    @property
    def state(self):
        ret = []
        vec = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                for k in [-1, 0, 1]:
                    if (i == 0 and j == 0 and k == 0): continue
                    vec = vector(i, j, k)
                    min_arm = self.bound_soner_arm(vec)
                    for idx in range(self.Natoms):
                        cur_arm = self.get_soner_arm(vec, self.player.pos, self.obs_pos[idx])
                        if (min_arm > cur_arm):
                            min_arm = cur_arm
                    ret.append(min_arm)
        return ret

    @property
    def N_STATES(self):
        return len(self.state)

    def step(self, dt, action):
        #rate(500)
        # check gameover
        self.gameOver = self.gameOver or self.checkGameOver()
        if (self.gameOver): return -1000, True
        mass = 1

        direction = self.action_dict[action]
        self.player.velocity = mag(self.player.velocity) * direction

        #update all positions
        for i in range(self.Natoms): self.obstructions[i].pos = self.obs_pos[i] = self.obs_pos[i] + (self.p[i]/mass)*dt
        self.player.pos = self.player.pos + self.player.velocity * dt
        
        # process collision
        hitlist = self.checkCollisions()
        for (i,j) in hitlist:
            ptot = self.p[i]+self.p[j]
            posi = self.obs_pos[i]
            posj = self.obs_pos[j]
            vi = self.p[i]/mass
            vj = self.p[j]/mass
            vrel = vj-vi
            a = vrel.mag2
            if a == 0: continue;  # exactly same velocities
            rrel = posi-posj
            if rrel.mag > self.Ratom: continue # one atom went all the way through another
        
            # theta is the angle between vrel and rrel:
            dx = dot(rrel, vrel.hat)       # rrel.mag*cos(theta)
            dy = cross(rrel, vrel.hat).mag # rrel.mag*sin(theta)
            # alpha is the angle of the triangle composed of rrel, path of atom j, and a line
            #   from the center of atom i to the center of atom j where atome j hits atom i:
            alpha = asin(dy/(2*self.Ratom)) 
            d = (2*self.Ratom)*cos(alpha)-dx # distance traveled into the atom from first contact
            deltat = d/vrel.mag         # time spent moving from first contact to position inside atom
            
            posi = posi-vi*deltat # back up to contact configuration
            posj = posj-vj*deltat
            mtot = 2*mass
            pcmi = self.p[i]-ptot*mass/mtot # transform momenta to cm frame
            pcmj = self.p[j]-ptot*mass/mtot
            rrel = norm(rrel)
            pcmi = pcmi-2*pcmi.dot(rrel)*rrel # bounce in cm frame
            pcmj = pcmj-2*pcmj.dot(rrel)*rrel
            self.p[i] = pcmi+ptot*mass/mtot # transform momenta back to lab frame
            self.p[j] = pcmj+ptot*mass/mtot
            self.obs_pos[i] = posi+(self.p[i]/mass)*deltat # move forward deltat in time
            self.obs_pos[j] = posj+(self.p[j]/mass)*deltat
        
        #process bouncing
        for i in range(self.Natoms):
            loc = self.obs_pos[i]
            if abs(loc.x) > 1/2:
                self.p[i].x =  - self.p[i].x
            
            if abs(loc.y) > 1/2:
                self.p[i].y = - self.p[i].y
            
            if abs(loc.z) > 1/2:
                self.p[i].z =  - self.p[i].z

        # check reward
        reward = self.checkReward()
        state = self.state

        return state, reward, self.gameOver


