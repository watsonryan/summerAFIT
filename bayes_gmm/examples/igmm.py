#!/usr/bin/env python

"""
IGMM to generate mixture-model for pose-graph optimization. 

"""
__author__ = 'ryan'
__email__ = "rwatso12@gmail.com"


import numpy as np
import matplotlib.pyplot as plt
import sys, random, logging, argparse

sys.path.append("..")


from bayes_gmm.niw import NIW
from bayes_gmm.igmm import IGMM
from plot_utils import plot_ellipse, plot_mixture_model

logging.basicConfig(level=logging.INFO)

random.seed(1)
np.random.seed(1)


def main():

  # Add command line interface
  parser = argparse.ArgumentParser(description="Wrapper script to generate "
                                  "Gaussian mixture-model for pose-graph optimization "   
                                  "using IGMM." )

  parser.add_argument('-i', '--inputFile', dest='input', 
                     default='../../poseGraphs/man35Faulty/100.res', 
                     help="Define the input pose graph residual file")
  parser.add_argument('-o', '--outFile', dest='output', 
                      help="Define the output file or output will print to stdout")
  parser.add_argument('-n', '--iterations', dest='iterations', default=10, type=int,
                      help="How many iterations of IGMM will be used to calculate mixture model?")
  parser.add_argument('-k', '--init_comp', dest='initComp', default=2, type=int,
                      help="How many inital components in model?")
  parser.add_argument('-a', '--alpha', dest='alpha', default=1.0, type=float,
                      help="What's the value of the Dirichlet hyper-parameter?")

  args = parser.parse_args()

  # Data parameters
  Data = np.loadtxt(args.input)
  D = min( Data.shape )           # dimensions

  # Intialize prior
  mu_scale = 1.
  covar_scale = 1.
  m_0 = np.zeros(D)
  k_0 = covar_scale**2/mu_scale**2
  v_0 = D + 3
  S_0 = covar_scale**2*v_0*np.eye(D)
  prior = NIW(m_0, k_0, v_0, S_0)

  # Setup IGMM
#    igmm = IGMM(Data, prior, args.alpha, assignments="rand", K=args.initComp)
  igmm = IGMM(Data, prior, args.alpha, assignments="one-by-one", K=args.initComp)

  # Perform Gibbs sampling
  record = igmm.gibbs_sample(args.iterations)

  # Plot results
  for k in xrange(igmm.components.K):
    mu, sigma = igmm.components.rand_k(k)
    print mu, sigma

if __name__ == "__main__":
    main()
