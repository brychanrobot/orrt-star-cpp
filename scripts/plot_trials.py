import json
import numpy as np
import matplotlib.pyplot as plt

j = json.load(open('scripts/fullTrials.json', 'r'))

obst = []
ave_score = []
for t in j[-1]['results']:
	obst.append(t['obstacles'])
	ave_score.append(np.mean(t['scores']))

plt.plot(obst, ave_score)
plt.show()