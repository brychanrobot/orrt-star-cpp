import json
import numpy as np
import matplotlib.pyplot as plt


def plot_paths(map, paths):
	#rr, cc = np.where(map == 1)

	#plt.scatter(cc, rr, marker=',', s=0.9, c='lightblue', lw=0)
	plt.imshow(map, zorder=0, extent=[0, map.shape[1], map.shape[0], 0], interpolation='nearest', cmap='BuPu')

	for p in paths:
		p = np.asarray(p[2])
		#print(p)
		plt.plot(p[:,0], p[:,1], linewidth=2, c='orange', zorder=1)

		plt.scatter(p[0, 0], p[0, 1], c='y', s=100, zorder=2)
		plt.scatter(p[-1, 0], p[-1, 1], c='r', s=100, zorder=2)

	plt.xlim((0, map.shape[1]))
	plt.ylim((0, map.shape[0]))
	plt.tight_layout()
	plt.show()

with open("data/60map.json", "r") as f:
	map = np.asarray(json.load(f))

with open("data/60paths.json", "r") as f:
	paths = json.load(f)

"""
rr, cc = np.where(map == 1)

#plt.scatter(cc, rr, marker=',', s=0.9)
plt.imshow(map, zorder=0, extent=[0, 50, 50, 0], interpolation='nearest', cmap='BuPu')

for path in paths[:1]:
	p = np.asarray(path[2])
	plt.plot(p[:,0], p[:,1])

plt.xlim((0, map.shape[1]))
plt.ylim((0, map.shape[0]))
plt.show()
"""
plot_paths(map, paths[20:21])
