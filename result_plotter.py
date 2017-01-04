import json
import glob
import numpy as np
import matplotlib.pyplot as plt

planner_types = {'astar_visibility' : 'A* w/ Visibility Graph', 'astar_grid' : 'A* w/ Grid', 'prmstar' : 'PRM*'}

frequencies = None
time_available = None

for planner_type in planner_types.keys():
	all_results = []
	for fname in glob.glob('data/*results*' + planner_type + '.json'):
		with open(fname, 'r') as f:
			j = json.load(f)
			all_results.append([r['timeused'] for r in j])
			frequencies = [r['frequency'] for r in j]
			time_available = [r['timeavailable'] for r in j]

	all_results = np.asarray(all_results)
	if len(all_results) > 0:
		percent_time_results = np.mean(all_results, axis=0)/np.asarray(time_available)
		print(frequencies)

		plt.plot(frequencies, percent_time_results, label=planner_types[planner_type])
		plt.legend(loc='upper left')

plt.ylim((0, 3))

plt.xlabel('# replans per second')
plt.ylabel('% computing time used')
plt.tight_layout()
plt.show()
