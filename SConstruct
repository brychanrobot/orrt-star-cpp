
libs= ['m', 'GL', 'GLU', 'glfw', 'glut',
'X11', 'Xrandr', 'Xinerama', 'Xi', 'Xxf86vm', 'Xcursor',
'pthread', 'dl' ,'boost_system']


env=Environment(CXXFLAGS='-Wall -O3 -march=native -std=c++11', LIBS=libs)
env.VariantDir('build', '.', duplicate=0)

objects = [
env.Object('build/geom/Rect.cpp'),

env.Object('build/Node.cpp'),

env.Object('build/Planner.cpp'),
env.Object('build/AStar.cpp'),

env.Object('build/SamplingPlanner.cpp'),
env.Object('build/OnlineFmtStar.cpp'),
env.Object('build/OnlineRrtStar.cpp')]

env.Program(['build/main.cpp'] + objects)
