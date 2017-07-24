#!/usr/bin/env python

'''
Scrit to generate mixture model from file of residuals
'''

__author__ = 'ryan'
__email__ = "rwatso12@gmail.com"

import warnings
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys, os, glob, subprocess, progressbar, argparse

sys.path.append("..")

from bayes_gmm.niw import NIW
from bayes_gmm.igmm import IGMM

# Used to check if covariance is P.D. before feeding into Max-Mix.
def is_pos_def(x):
  return np.all(np.linalg.eigvals(x) > 0)


def main(argv):

  warnings.filterwarnings("ignore")

  # Add command line interface
  parser = argparse.ArgumentParser(description="Scrit to test pose graph  "
                                  "optimization using Max-Mixtures robust noise"
                                  "model with E.M. estimated mixture components")

  parser.add_argument('-i', '--inputFile', dest='input',
                     default='../../poseGraphs/manhattanOlson3500.g2o',
                     help="Define the input pose graph")
  parser.add_argument('-a', '--alpha', dest='alpha', default=1e-6, type=float,
                      help="What's the value of the Dirichlet hyper-parameter?")
  parser.add_argument('-n', '--iterations', dest='iterations', default=10, type=int,
                      help="How many iterations of IGMM will be used to calculate mixture model?")
  parser.add_argument('-k', '--init_comp', dest='initComp', default=2, type=int,
                      help="How many inital components in model?")
  args = parser.parse_args()

  ##############################################################################
  # 2) Use IGMM To Calculate Mixture Model
  ##############################################################################

  residuals = np.loadtxt(args.input)
  D = min( residuals.shape )
  # Intialize prior
  mu_scale = 1.
  covar_scale = 1.
  m_0 = np.zeros(D)
  k_0 = covar_scale**2/mu_scale**2
  v_0 = D + 3
  S_0 = covar_scale**2*v_0*np.eye(D)
  prior = NIW(m_0, k_0, v_0, S_0)

  # Setup IGMM
  igmm = IGMM(residuals, prior, args.alpha, assignments="rand", K=args.initComp)

  # Perform Gibbs sampling
  record = igmm.gibbs_sample(args.iterations)

  # Plot results
  mixture = np.array([]).reshape(0,9)
  for k in xrange(igmm.components.K):
    mu, sigma = igmm.components.rand_k(k)
    if ( is_pos_def(sigma) ):
      mixture =  np.vstack( [mixture, np.reshape( sigma, ( 1, sigma.size ) ) ] )
    else:
      print " Non P.D. Matrix " , sigma 
  mixture = np.array(sorted(mixture,key=tuple))
  np.savetxt( 'mixtureModel.txt', mixture, delimiter=',' )

if __name__=="__main__":
  main(sys.argv[1:])
