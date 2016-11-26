
libs= ['m', 'GL', 'GLU', 'glfw', 'glut',
'X11', 'Xrandr', 'Xinerama', 'Xi', 'Xxf86vm', 'Xcursor',
'pthread', 'dl', 'boost_system', 'freeimage']


env=Environment(CXXFLAGS='-Wall -O3 -march=native -std=c++11', LIBS=libs)
debug = ARGUMENTS.get('debug', 0)
if int(debug):
	env.Append(CCFLAGS = '-g')

env.VariantDir('build', 'src', duplicate=0)

objects = [
	env.Object('build/geom/Rect.cpp'),

	env.Object('build/planning/Node.cpp'),

	env.Object('build/planning/Planner.cpp'),
	env.Object('build/planning/AStar.cpp'),

	env.Object('build/planning/SamplingPlanner.cpp'),
	env.Object('build/planning/OnlineFmtStar.cpp'),
	env.Object('build/planning/OnlineRrtStar.cpp')
]

env.Program(['build/main.cpp'] + objects)
env.Program(['build/generatePaths.cpp'] + objects)
