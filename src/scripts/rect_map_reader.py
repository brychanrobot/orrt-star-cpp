import json
import numpy as np
from bokeh.plotting import figure, show

"""
def plot_paths(conf_space, paths):
	#rr, cc = np.where(conf_space == 1)

	#plt.scatter(cc, rr, marker=',', s=0.9, c='lightblue', lw=0)
	plt.imshow(conf_space, zorder=0, extent=[0, conf_space.shape[1], conf_space.shape[0], 0], interpolation='nearest', cconf_space='BuPu')

	for p in paths:
		p = np.asarray(p[2])
		#print(p)
		plt.plot(p[:,0], p[:,1], linewidth=2, c='orange', zorder=1)

		plt.scatter(p[0, 0], p[0, 1], c='y', s=100, zorder=2)
		plt.scatter(p[-1, 0], p[-1, 1], c='r', s=100, zorder=2)

	plt.xlim((0, conf_space.shape[1]))
	plt.ylim((0, conf_space.shape[0]))
	plt.tight_layout()
	plt.show()
"""

def plot_conf_space(conf_space):
	p = figure(x_range=(0, conf_space['width']), y_range=(0, conf_space['height']))
	obst = np.asarray(conf_space['obstacles'])

	p.quad(top=obst[:, 0, 1] + 5, left=obst[:, 0, 0] + 5, bottom=obst[:, 1, 1] - 5, right=obst[:, 1, 0] - 5)
	return p

def plot_paths(p, paths):
	for path in paths:
		waypoints = np.asarray(path['path'])
		p.line(waypoints[:, 0], waypoints[:, 1])

	show(p)

with open('data/map0.json', 'r') as f:
	conf_space = json.load(f)

with open('data/map0replan.json', 'r') as f:
	paths = json.load(f)

p = plot_conf_space(conf_space)
plot_paths(p, paths)
