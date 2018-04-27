import matplotlib.pyplot as plt
import numpy as np

naive_dat = np.genfromtxt('../results/ResultsNaive.csv', delimiter=',')
interp_dat = np.genfromtxt('../results/ResultsInterp.csv', delimiter=',')

def err(d):
  diff = d[5:8] - d[2:5]
  return np.sqrt(np.sum(diff**2))

shift = -10
plt.plot(naive_dat[1000:1100,0], naive_dat[1000:1100,6])
plt.plot(interp_dat[1000:1100,0], interp_dat[(1000-shift):(1100-shift),6])
plt.plot(interp_dat[1000:1100,0], interp_dat[(1000-shift):(1100-shift),6])
#plt.plot(naive_dat[:,0], naive_dat[:,6])
#plt.plot(interp_dat[:,0], interp_dat[:,6])
plt.show()

print(naive_dat.shape)
print(interp_dat.shape)
end_indx = min(naive_dat.shape[0]+shift, interp_dat.shape[0]+shift)-400
end_indxB = end_indx - shift
eq_indx = end_indx - 1000
eq_indxB = eq_indx - shift
time_from_eq = (naive_dat[eq_indx:end_indx,1]-naive_dat[eq_indx,1])
plt.plot(time_from_eq, [err(d) for d in naive_dat[eq_indx:end_indx,]], label="naive")
plt.legend()
plt.xlabel("Time (s)")
plt.ylabel("Error (m)")
plt.ylim((0,0.035))
plt.show()

plt.plot(time_from_eq, [err(d) for d in naive_dat[eq_indx:end_indx,]], label="naive")
plt.plot(time_from_eq, [err(d) for d in interp_dat[eq_indxB:end_indxB,]], label="forecast")
plt.legend()
plt.xlabel("Time (s)")
plt.ylabel("Error (m)")
plt.ylim((0,0.035))
plt.show()
