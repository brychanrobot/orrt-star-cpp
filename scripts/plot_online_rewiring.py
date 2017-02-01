from operator import itemgetter
import json
import numpy as np
import matplotlib.pyplot as plt

j = json.load(open('scripts/onlineRewiring.json', 'r'))

for spot in j:
	results = spot['results']
	radius = spot['radius']
	plt.plot([result['time'] for result in results], [result['entropy'] for result in results], label='radius=%.2f' % radius)

plt.xlabel('time (s)')
plt.ylabel('entropy')
plt.legend(loc='lower left')



plt.figure()

final_entropies = []
radii = []

for spot in j:
	results = spot['results']
	radii.append(spot['radius'])
	final_entropies.append(results[-1]['entropy'])

plt.plot(radii, final_entropies)
plt.xlabel('radius')
plt.ylabel('entropy')

plt.show()