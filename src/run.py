import subprocess
import math
import sys

min_result =  math.inf
for x in range(1, 100):
	result = subprocess.run(['./a.out', sys.argv[1], '1', '96', '0'], stdout=subprocess.PIPE)
	if min_result > int(result.stdout):
		min_result = int(result.stdout)
print( min_result )
