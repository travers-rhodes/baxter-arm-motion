import scipy.io
import numpy as np


for i in xrange(1,9):
    fname = 'data%d.npy'%i
    print fname
    data = np.load(fname)
    scipy.io.savemat('data%d.mat'%i, dict(data=data))
