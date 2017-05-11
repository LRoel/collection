from numpy.random import randn
import copy
class PosSenser1(object):
    def __init__(self,pos=(0,0),vel=(0,0),noise_std=1.):
        self.vel = vel
        self.noise_std = noise_std
        self.pos = [pos[0],pos[1]]

    def read(self):
        self.pos[0] += self.vel[0]
        self.pos[1] += self.vel[1]

        return [self.pos[0] + randn() * self.noise_std,self.pos[1] + randn()*self.noise_std]