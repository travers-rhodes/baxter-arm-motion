import numpy as np


class Model(object):
    origin = np.array([0.6720409, -0.72401, 0.0569])
    def get_position(self, time):
        print('ERROR UNIMPLEMENTED')
    def generate_data(self, tFinal, sampleRate):
        p = np.array([0, 0, 0])
        data = np.zeros((nSamples, 3))
        nSamples = tFinal*sampleRate
        for i in range(nSamples):
            time = float(i)*tFinal/float(nSamples)
            p = self.get_position(time)
            data[i, :] = p + np.random.normal(0, 0.001, size=(1, 3))
        return data
    def generator(self):
        p = np.array([0.0, 0.0, 0.0])
        try:
            i=0
            while True:
                p = self.get_position(p, i)
                yield p + np.random.normal(0, 0.03, size=(1, 3))
        except:
            yield p
            print('Generator exited')

            
# class LinearModel(Model):
#     def __init__(self, direction):
#         self.direction = np.array(direction)
#     def get_position(self, time):
#         q = np.copy(p)
#         q = q + self.direction
#         return q

class Ellipse(Model):
    def __init__(self, a = 0.1, b = 0.1):
        self.a = a
        self.b = b
        
    def get_position(self, time):
        theta = time*np.pi*2/8
        q = np.array([0, self.a*np.cos(theta), self.b*np.sin(theta)]) + self.origin
        return q

class FigureEight(Model):
    def __init__(self, a = 0.1, b = 0.1):
        self.a = a
        self.b = b
        
    def get_position(self, time):
        theta = 2*np.pi*time/8
        q = np.array([0, self.a*np.cos(2*theta + np.pi/2), self.b*np.sin(theta)]) + self.origin
        return q

class Snake(Model):
    def __init__(self, a = 0.1, b = 0.1):
        self.a = a
        self.b = b
        
    def get_position(self, time):
        theta = 2*np.pi*time/8
        q = np.array([0, self.a*np.cos(4*theta), self.b*np.sin(theta+np.pi/2)]) + self.origin
        return q



