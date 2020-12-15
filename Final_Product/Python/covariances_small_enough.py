# Harrison Hidalgo
# ECE 5725
# This script returns True if covariances are small enough.

def covariances_small_enough(S,min_covariances):
	import numpy as np
	import math
	P = np.matmul(S,np.transpose(S))
	is_it = True
	for i in range(0,len(S)):
		if min_covariances[i] < math.sqrt(P[i,i]):
			is_it=False
	return is_it
