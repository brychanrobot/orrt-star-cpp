import json
import numpy as np
import matplotlib.pyplot as plt

j = json.load(open('scripts/fullTrials.json', 'r'))

for scenario in j[-5:]:
	obst = []
	ave_score = []
	for t in scenario['results']:
		obst.append(t['obstacles'])
		ave_score.append(np.mean(t['scores']))

	plt.plot(obst, ave_score, label=scenario['scenario'])
	plt.xticks(obst, map(str, obst))
plt.xlabel('# obstacles')
plt.ylabel('average score')
plt.legend()
plt.show()