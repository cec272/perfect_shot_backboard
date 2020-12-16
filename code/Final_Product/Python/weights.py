# Harrison Hidalgo
# ECE 5725 - Final Project
# These are the weights for the sigma point filter
import math

# Important numbers
nx    = 6 # Number of states
n_sig = 1 # Number of sigma points
## State weights
wm  = 1/2/math.pow(n_sig,2)
wm0 = (math.pow(n_sig,2)-nx)/math.pow(n_sig,2)
## Covariance weights
wc  = 1/2/math.pow(n_sig,2)
wc0 = (math.pow(n_sig,2)-nx)/math.pow(n_sig,2)-math.pow(n_sig,2)/nx+3

