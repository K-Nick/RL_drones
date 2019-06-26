#%% 载入模组
from vpython import *
import os
import numpy as np

#%% 定义
class Space(object):
    def __init__(self, display=False):
        # set parameters
        win = 700
        self.display = display
        self.Natoms = 15

        L = 1
        gray = color.gray(0.7)
        self.mass = 1
        self.Ratom = 0.03
        self.level = 0
        self.dist = [0.6, 0.4, 0.2, 0.1, 0.01]
        self.reward = [100, 200, 300, 600, 1000]
        self.init_pos = vector(0.45, 0.45, 0.45)
        self.target_pos = vector(-0.45, -0.45, -0.45)
        self.gameOver = False

        #create canvas
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
            theta = pi*random()
            phi = 2*pi*random()
            px = pavg*sin(theta)*cos(phi)
            py = pavg*sin(theta)*sin(phi)
            pz = pavg*cos(theta)
            self.p.append(vector(px, py, pz))

    def reset(self):
        self.__init__(display=self.display)
    
    def checkReward(self):
        cur_threshold = self.dist[self.level]
        d2 = cur_threshold ** 2
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
        d2 = (self.Ratom + self.Ratom * 2) **2
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
        d2 = (self.Ratom * 2) ** 2
        for i in range(self.Natoms):
            posx = self.obs_pos[i]
            for j in range(i):
                posy = self.obs_pos[j]
                pos_delta = posy - posx
                if (mag2(pos_delta) < d2):
                    hitlist.append((i,j))
        
        return hitlist

    @property
    def state(self):
        ret = []
        for i in range(self.Natoms):
            ret += self.obs_pos[i].value
            ret += self.p[i].value
        ret += self.player.pos.value
        ret += self.player.velocity.value
        return ret
    
    @property
    def observation_size(self):
        return 3 * (2 * self.Natoms + 2)

    def step(self, dt, action):
        rate(500)
        # check gameover
        self.gameOver = self.gameOver or self.checkGameOver()
        if (self.gameOver): return -1000, True
        mass = 1

        action = action / np.sqrt(mag2(action))
        self.player.velocity = np.sqrt(mag2(self.player.velocity)) * action

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

if __name__ == "__main__":
    space = Space(display=True)
    dt = 1e-3
    T = 10
    t = 0
    while(True):
        if (space.gameOver or t >= T):
            space.reset()
            print("\ngame over\n")
            os.system("pause")
            t = 0
        while t < T:
            state, reward, gameOver = space.step(dt, vector(-1, -1, -1))
            print(len(state), space.observation_size)
            t += dt


#%%
