import numpy as np


class Model(object):
    def compute_step(self, p, i):
        print('ERROR UNIMPLEMENTED')
    def generate_data(self, N):
        p = np.array([0, 0, 0])
        data = np.zeros((N, 3))
        for i in range(N):
            p = self.compute_step(p, i)
            data[i, :] = p + np.random.normal(0, 0.05, size=(1, 3))
        return data

class LinearModel(Model):
    def __init__(self, direction):
        self.direction = np.array(direction)
    def compute_step(self, p, i):
        q = np.copy(p)
        q = q + self.direction
        return q

class Ellipse(Model):
    def __init__(self, a, b):
        self.a = a
        self.b = b
        
    def compute_step(self, p, i):
        theta = i/(2*np.pi*100)
        q = np.array([self.a*np.cos(theta), self.b*np.size(theta), 0])
        return q

class FigureEight(Model):
    def __init__(self, a, b):
        self.a = a
        self.b = b
        
    def compute_step(self, p, i):
        theta = i/(2*np.pi*100)
        q = np.array([self.a*np.cos(2*theta), self.b*np.size(theta), 0])
        return q



