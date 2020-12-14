# Harrison Hidalgo
# ECE 5725 - Final Project
# These are the weights for the sigma point filter

# Important numbers
nx    = 6 # Number of states
n_sig = 1 # Number of sigma points
## State weights
wm  = 1/2/n_sig^2
wm0 = (n_sig^2-nx)/n_sig^2
## Covariance weights
wc  = 1/2/n_sig^2
wc0 = (n_sig^2-nx)/n_sig^2-n_sig^2/n+3

