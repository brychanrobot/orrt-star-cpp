import subprocess
import time

for obstacles in [5, 10, 15, 20]:

	scores = []

	for i in range(30):
		procs = []
		for j in range(4):
			time.sleep(0.1) # allow for different random number generation
			procs.append(subprocess.Popen(['build/main', '-w', '15' , '-l', '180', '-o', str(obstacles), '--fmt'], stdout=subprocess.PIPE))

		for proc in procs:
			proc.wait()
			try:
				line = proc.stdout.readline()
				scores.append(float(line))
			except ValueError:
				print("invalid value: %s" % line)
		
		print('%d completed' % (i * 4))

	print("obstacles: %d" % obstacles)
	print(scores)

print('finished')