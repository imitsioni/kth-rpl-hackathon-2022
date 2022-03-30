import psutil

PROCNAME = "./basic_folding.py"
for proc in psutil.process_iter():
	# check whether the process name matches
	if 'python' in proc.name():
		print(proc.name())
		import ipdb
		ipdb.set_trace()
	if proc.name() == PROCNAME:
		proc.kill()
