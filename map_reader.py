import json
import numpy as np
import matplotlib.pyplot as plt

with open("data/1map.json", "r") as f:
	map = np.asarray(json.load(f))

with open("data/1paths.json", "r") as f:
	paths = json.load(f)

rr, cc = np.where(map == 1)

plt.scatter(cc, rr)

for path in paths:
	p = np.asarray(path[2])
	plt.plot(p[:,0], p[:,1])

plt.xlim((0, 700))
plt.ylim((0, 700))
plt.show()
