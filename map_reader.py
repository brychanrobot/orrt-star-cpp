import json
import numpy as np
import matplotlib.pyplot as plt

with open("data/60map.json", "r") as f:
	map = np.asarray(json.load(f))

with open("data/60paths.json", "r") as f:
	paths = json.load(f)

rr, cc = np.where(map == 1)

#plt.scatter(cc, rr, marker=',', s=0.9)
plt.imshow(map, zorder=0, extent=[0, 50, 50, 0], interpolation='nearest', cmap='BuPu')

for path in paths[:1]:
	p = np.asarray(path[2])
	plt.plot(p[:,0], p[:,1])

plt.xlim((0, map.shape[1]))
plt.ylim((0, map.shape[0]))
plt.show()
