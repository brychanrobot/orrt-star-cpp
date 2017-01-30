import subprocess
import time

for i in range(5, 40):
	proc = subprocess.Popen(['build/main', '-w', '15' , '-l', '2000', '-o', '0', 'p', str(i)], stdout=subprocess.PIPE)
	proc.wait()

	results = proc.stdout.read().splitlines()
	
	print('{"radius": %d, "results": %s},' % (i, results))