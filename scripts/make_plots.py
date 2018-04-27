import matplotlib.pyplot as plt
import numpy as np

naive_dat = np.genfromtxt('../results/ResultsNaive.csv', delimiter=',')
interp_dat = np.genfromtxt('../results/ResultsInterp.csv', delimiter=',')

def err(d):
  diff = d[5:8] - d[2:5]
  return np.sqrt(np.sum(diff**2))

eq_indx = 200
print(naive_dat.shape)
print(interp_dat.shape)
end_indx = min(naive_dat.shape[0], interp_dat.shape[0]) 
time_from_eq = (naive_dat[eq_indx:end_indx,0]-naive_dat[eq_indx,0])/1000000.0
plt.plot(time_from_eq, [err(d) for d in naive_dat[eq_indx:end_indx,]])
plt.show()

eq_indx = 200
plt.plot(time_from_eq, [err(d) for d in naive_dat[eq_indx:end_indx,]])
plt.plot(time_from_eq, [err(d) for d in interp_dat[eq_indx:end_indx,]])
plt.show()
